#include "roboteam_world/danger_finder/CanShootModule.h"
#include "roboteam_utils/world_analysis.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/LastWorld.h"

namespace rtt {
namespace df {

REGISTER_MODULE("CanShoot", CanShootModule)

CanShootModule::CanShootModule() : DangerModule("CanShoot") {}

PartialResult CanShootModule::calculate(const roboteam_msgs::WorldRobot& bot, const roboteam_msgs::World& world) {
	Vector2 botPos(bot.pos);
	Vector2 goalPos = LastWorld::get_our_goal_center();

	auto obstacles = getObstaclesBetweenPoints(botPos, goalPos);
	if (obstacles.size() > 0 && bot_has_ball(bot, world.ball)) {
		return { myConfig().doubles["canShootDanger"], DANGER_CAN_SHOOT };
	}
	return PartialResult();
}

}
}