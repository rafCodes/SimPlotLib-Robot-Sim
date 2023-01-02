#!/usr/bin/env python
import random
if __name__ == '__main__':
    goal = {
        1: [-135, 1.50],
        2: [-212.00, -73.95],
        3: [-212.00, -198.22],
        4: [-135.50, -256.00],
        5: [-84.5, -198.22],
        6: [-135.50, -128.00],
        7: [-135.50, -48.20],
        8: [44.50, -65.75],
        9: [0.0, -128.00],
        10: [44.50, -198.22],
        11: [0.0, -256.00]}

    # goalOrder = [1,2,3,4,5,6,7,8,9,10,11]
    goalOrder = [1, 2, 3, 4, 5, 9, 8, 7, 6, 10, 11]  # best path
    goalOrder = [1, 7, 2, 3, 4, 5, 6, 9, 8, 10, 11]  # best back path
    goalOrder = [8, 9, 10, 11, 4, 5, 6, 3, 2, 7, 1]

    # goalOrder *= 3
    # random.shuffle(goalOrder)
    goals = []
    for i in goalOrder:
        goals.append(goal[i])
    print(goals)
