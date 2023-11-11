from typing import List

from aido_schemas import Context, FriendlyPose
from dt_protocols import (
    PlacedPrimitive,
    PlanningQuery,
    PlanningResult,
    PlanningSetup,
    PlanStep,
    Circle,
    Rectangle,
    SimulationResult,
    simulate,
)

__all__ = ["Planner"]


class Planner:
    params: PlanningSetup

    def init(self, context: Context):
        context.info("init()")

    def on_received_set_params(self, context: Context, data: PlanningSetup):
        context.info("initialized")
        self.params = data

        # This is the interval of allowed linear velocity
        # Note that min_velocity_x_m_s and max_velocity_x_m_s might be different.
        # Note that min_velocity_x_m_s may be 0 in advanced exercises (cannot go backward)
        max_velocity_x_m_s: float = self.params.max_linear_velocity_m_s
        min_velocity_x_m_s: float = self.params.min_linear_velocity_m_s

        # This is the max curvature. In earlier exercises, this is +inf: you can turn in place.
        # In advanced exercises, this is less than infinity: you cannot turn in place.
        max_curvature: float = self.params.max_curvature

        # these have the same meaning as the collision exercises
        body: List[PlacedPrimitive] = self.params.body
        environment: List[PlacedPrimitive] = self.params.environment

        # these are the final tolerances - the precision at which you need to arrive at the goal
        tolerance_theta_deg: float = self.params.tolerance_theta_deg
        tolerance_xy_m: float = self.params.tolerance_xy_m

        # For convenience, this is the rectangle that contains all the available environment,
        # so you don't need to compute it
        bounds: Rectangle = self.params.bounds

    def on_received_query(self, context: Context, data: PlanningQuery):
        # A planning query is a pair of initial and goal poses
        start: FriendlyPose = data.start
        goal: FriendlyPose = data.target

        # You start at the start pose. You must reach the goal with a tolerance given by
        # tolerance_xy_m and tolerance_theta_deg.

        # You need to declare if it is feasible or not
        feasible = True

        if not feasible:
            # If it's not feasible, just return this.
            result: PlanningResult = PlanningResult(False, None)
            context.write("response", result)
            return

        # If it is feasible you need to provide a plan.

        # A plan is a list of PlanStep
        plan: List[PlanStep] = []

        # A plan step consists in a duration, a linear and angular velocity.

        # For now let's just trace a square of side L at maximum velocity.
        L = 1.0
        duration_straight_m_s = L / self.params.max_linear_velocity_m_s
        duration_turn_deg_s = 90.0 / self.params.max_angular_velocity_deg_s
        # The plan will be: straight, turn, straight, turn, straight, turn, straight, turn

        straight = PlanStep(
            duration=duration_straight_m_s,
            angular_velocity_deg_s=0.0,
            velocity_x_m_s=self.params.max_linear_velocity_m_s,
        )
        turn = PlanStep(
            duration=duration_turn_deg_s,
            angular_velocity_deg_s=self.params.max_angular_velocity_deg_s,
            velocity_x_m_s=0.0,
        )

        plan.append(straight)
        plan.append(turn)
        plan.append(straight)
        plan.append(turn)
        plan.append(straight)
        plan.append(turn)
        plan.append(straight)
        plan.append(turn)

        result: PlanningResult = PlanningResult(feasible, plan)
        context.write("response", result)
