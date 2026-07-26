from robot320_localization_bringup.frontier import choose_frontier, find_frontiers


def test_find_frontiers_clusters_free_unknown_boundary():
    width = 9
    height = 9
    data = [-1] * (width * height)
    for row in range(2, 7):
        for column in range(2, 7):
            data[row * width + column] = 0

    frontiers = find_frontiers(
        data,
        width,
        height,
        min_size=4,
        clearance_cells=0,
    )

    assert len(frontiers) == 1
    assert frontiers[0].size == 16
    assert 2 <= frontiers[0].row <= 6
    assert 2 <= frontiers[0].column <= 6


def test_clearance_rejects_frontier_next_to_obstacle():
    width = 7
    height = 7
    data = [-1] * (width * height)
    for row in range(1, 6):
        for column in range(1, 6):
            data[row * width + column] = 0
    for row, column in ((1, 2), (1, 3), (1, 4), (2, 1), (3, 1), (4, 1)):
        data[row * width + column] = 100

    frontiers = find_frontiers(
        data,
        width,
        height,
        min_size=2,
        clearance_cells=1,
    )

    assert frontiers
    assert all(data[item.row * width + item.column] == 0 for item in frontiers)


def test_choose_frontier_balances_distance_and_information_gain():
    from robot320_localization_bringup.frontier import Frontier

    near = Frontier(row=2, column=2, size=4)
    far_large = Frontier(row=5, column=5, size=100)

    assert choose_frontier([near, far_large], 0, 0) == near
    assert (
        choose_frontier(
            [near, far_large],
            0,
            0,
            information_gain_weight=1.0,
        )
        == far_large
    )
