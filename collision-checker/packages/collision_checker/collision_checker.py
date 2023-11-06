import itertools
import random
from typing import List

from aido_schemas import Context, FriendlyPose
from dt_protocols import (
    Circle,
    CollisionCheckQuery,
    CollisionCheckResult,
    MapDefinition,
    PlacedPrimitive,
    Rectangle,
)

__all__ = ["CollisionChecker"]


class CollisionChecker:
    params: MapDefinition

    def init(self, context: Context):
        context.info("init()")

    def on_received_set_params(self, context: Context, data: MapDefinition):
        context.info("initialized")
        self.params = data

    def on_received_query(self, context: Context, data: CollisionCheckQuery):
        collided = check_collision(
            environment=self.params.environment, robot_body=self.params.body, robot_pose=data.pose
        )
        result = CollisionCheckResult(collided)
        context.write("response", result)


def check_collision(
    environment: List[PlacedPrimitive], robot_body: List[PlacedPrimitive], robot_pose: FriendlyPose
) -> bool:
    # This is just some code to get you started, but you don't have to follow it exactly

    # TODO you can start by rototranslating the robot_body by the robot_pose
    rototranslated_robot: List[PlacedPrimitive] = []

    # Then, call check_collision_list to see if the robot collides with the environment
    collided = check_collision_list(rototranslated_robot, environment)

    # TODO return the status of the collision
    # for now let's return a random guess
    return random.uniform(0, 1) > 0.5


def check_collision_list(
    rototranslated_robot: List[PlacedPrimitive], environment: List[PlacedPrimitive]
) -> bool:
    # This is just some code to get you started, but you don't have to follow it exactly
    for robot, envObject in itertools.product(rototranslated_robot, environment):
        if check_collision_shape(robot, envObject):
            return True

    return False


def check_collision_shape(a: PlacedPrimitive, b: PlacedPrimitive) -> bool:
    # This is just some code to get you started, but you don't have to follow it exactly

    # TODO check if the two primitives are colliding
    if isinstance(a.primitive, Circle) and isinstance(b.primitive, Circle):
        ...
    if isinstance(a.primitive, Rectangle) and isinstance(b.primitive, Circle):
        ...
    if isinstance(a.primitive, Rectangle) and isinstance(b.primitive, Rectangle):
        ...
    ...

    # TODO return the status of the collision
    # for now let's return a random guess
    return random.uniform(0, 1) > 0.5
