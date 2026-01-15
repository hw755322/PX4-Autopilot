/****************************************************************************
 *
 *   Copyright (c) 2012-2022 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/time.h>

#include <uORB/uORB.h>
#include <uORB/topics/mavlink_heartbeat.h>

#include <poll.h>

class AliveHeartbeat : public ModuleBase<AliveHeartbeat>
{
	public :
		static int task_spawn(int argc, char *argv[])
		{
			_task_id = px4_task_spawn_cmd("alive_heartbeat",
						      SCHED_DEFAULT,
						      SCHED_PRIORITY_DEFAULT,
						      2000,
						      (px4_main_t)&run_trampoline,
						      (char *const *)argv);
			return (_task_id < 0) ? -errno : 0;
		}
		static AliveHeartbeat *instantiate(int argc, char *argv[])
		{
			(void)argc; (void)argv;
			return new AliveHeartbeat();
		}
		static int print_usage(const char *reason = nullptr)
		{
			if (reason) {
				PX4_WARN("%s", reason);
			}

			PRINT_MODULE_DESCRIPTION(
				R"DESCRTR(
				Listen to uORB topic mavlink_heartbeat and periodically print "I'm alive!" messages.")DESCRTR");

				PRINT_MODULE_USAGE_NAME("alive_heartbeat", "template");
				PRINT_MODULE_USAGE_COMMAND("start");
				PRINT_MODULE_USAGE_COMMAND("stop");
				PRINT_MODULE_USAGE_COMMAND("status");
				return 0;
		}
		static int custom_command(int argc, char *argv[])
		{
			return print_usage("unrecognized command");
		}

		void run() override
		{
			int hb_sub = orb_subscribe(ORB_ID(mavlink_heartbeat));

			mavlink_heartbeat_s hb{};
			orb_copy(ORB_ID(mavlink_heartbeat), hb_sub, &hb);

			pollfd fds{};
			fds.fd = hb_sub;
			fds.events = POLLIN;

			uint64_t count = 0;

			PX4_INFO("alive_heartbeat started (listening mavlink_heartbeat)");

			while (!should_exit()) {
				const int pret = ::poll(&fds, 1, 1000);

				if (pret > 0 && (fds.revents & POLLIN)) {
					bool updated = false;
					orb_check(hb_sub, &updated);

					if (updated) {
						orb_copy(ORB_ID(mavlink_heartbeat), hb_sub, &hb);
						count++;

						PX4_INFO("I'm alive! count=%llu sysid=%u compid=%u type=%u",
							(unsigned long long) count,
							hb.sysid, hb.compid, hb.type);

					}
				} else if (pret < 0) {
					PX4_ERR("poll error (%d)", errno);
					px4_usleep(100000);
				}
			}

			orb_unsubscribe(hb_sub);
			PX4_INFO("alive_heartbeat stopped");
		}
};
extern "C" __EXPORT int alive_heartbeat_main(int argc, char *argv[]){
	return AliveHeartbeat::main(argc, argv);
	}
