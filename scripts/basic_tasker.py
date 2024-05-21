
from math import inf

class BasicTasker:

    def __init__(self, robots) -> None:
        self.robots = robots

    def closest_goals(self):
        closest_goals = []

        for robot in robots:
            # calculate distances
            distances = robot['distance']
            closest_goal = inf
            closest_index = 0
            for index, distance in enumerate(distances):
                if distance < closest_goal:
                    closest_goal = distance
                    closest_index = index
            closest_goals.append(closest_index)

        return closest_goals


if __name__ == '__main__':
    robots = [
        {'namespace':'/tb1','distance':[1,2,3,4,5]},
        {'namespace':'/tb2','distance':[2,4,5,1,3]}
    ]

    tasker = BasicTasker(robots)
    assigned_goals = tasker.closest_goals()
    print(assigned_goals)
