import math

import pandas as pd


def load_data():
    data = pd.read_csv("Showcase Names - Sheet1.csv")

    try:
        from dancer_dupe_map import dancer_dupe_map
    except ImportError:
        dancer_dupe_map = {}

    routine_map = {
        # "Column1": "PT Contemp",
    }

    routines: dict[str, list[str]] = {
        routine_map.get(routine.strip(), routine.strip()): sorted(
            dancer_dupe_map.get(dancer.strip(), dancer.strip())
            for dancer in data[routine]
            if not (isinstance(dancer, float) and math.isnan(dancer))
        )
        for routine in sorted(data.columns)
    }
    routines["[Intermission]"] = []
    dancers = sorted(set(dancer for dancers in routines.values() for dancer in dancers))

    import difflib

    possible_dupes: list[tuple[str, list[str]]] = []
    for i, d1 in enumerate(dancers):
        close_matches = difflib.get_close_matches(d1, dancers[i + 1 :])
        if close_matches:
            possible_dupes.append((d1, close_matches))

    return routines, dancers, possible_dupes


def main():
    # Data loading and summary statistics
    routines, dancers, possible_dupes = load_data()

    print(f"{len(routines) - 1} routines:")
    for routine in sorted(routines.keys(), key=lambda routine: (-len(routines[routine]), routine)):
        print(f"    {routine:<20}  with {len(routines[routine])} dancer(s)")

    print(f"{len(dancers)} dancers:")
    dancer_routine_counts = {dancer: sum((dancer in dancers) for dancers in routines.values()) for dancer in dancers}
    for dancer in sorted(dancers, key=lambda dancer: (-dancer_routine_counts[dancer], dancer)):
        print(f"    {dancer:<20}  in {dancer_routine_counts[dancer]} routine(s)")

    # CP-SAT

    num_slots = 31 + 1  # +1 for intermission

    from collections import defaultdict

    from ortools.sat.python import cp_model

    model = cp_model.CpModel()

    ### Variables ###

    # A boolean flag for each (routine, slot) pair, indicating whether that routine is in that slot.
    routine_slot_flags = {
        routine: [model.new_bool_var(f"[{routine}] in slot {i}") for i in range(num_slots)]
        for routine in routines.keys()
    }

    # An integer for each (dancer, dancer's routine) pair, indicating how many slots have passed since that dancer's last routine.
    dancer_wait_times = {
        dancer: {
            routine: model.new_int_var(
                # 0, num_slots - dancer_routine_counts[dancer],
                1,
                2,
                f"[{dancer}] wait time for [{routine}]",
            )
            for routine, dancers in routines.items()
            if dancer in dancers
        }
        for dancer in dancers
        if dancer_routine_counts[dancer] >= 2  # No need to track wait times for dancers who are only in 1 routine.
    }

    ### Constraints ###

    # Each routine is in exactly one slot.
    for routine in routines.keys():
        model.add_exactly_one(routine_slot_flags[routine])

    # Each slot has 1 or 2 routines.
    slot_routine_count = [sum(routine_slot_flags[routine][i] for routine in routines.keys()) for i in range(num_slots)]
    for i in range(num_slots):
        model.add(slot_routine_count[i] >= 1)
        model.add(slot_routine_count[i] <= 2)

    # If intermission or a routine with more than 2 dancers is in a slot, no other routines can share that slot.
    for routine, dancers_in_routine in routines.items():
        if len(dancers_in_routine) == 0 or len(dancers_in_routine) > 2:
            for other_routine in routines.keys():
                if other_routine != routine:
                    for i in range(num_slots):
                        model.add_implication(routine_slot_flags[routine][i], ~routine_slot_flags[other_routine][i])

    for dancer, dancer_routines in dancer_wait_times.items():
        for routine, wait_time in dancer_routines.items():
            for i in range(num_slots):
                for j in range(i):
                    # If the routine is in slot i, and the dancer is in slot j,
                    # and intermission is not between slot j and slot i,
                    # then the wait time for this routine is <= i - j - 1.
                    for other_routine in dancer_routines.keys():
                        if other_routine == routine:
                            continue
                        model.add(wait_time <= i - j - 1).only_enforce_if(
                            routine_slot_flags[routine][i],
                            routine_slot_flags[other_routine][j],
                            *[~routine_slot_flags["[Intermission]"][k] for k in range(j + 1, i)],
                        )

    def require_precedes(routine1: str, routine2: str):
        for i in range(num_slots - 1):
            model.add(routine_slot_flags[routine1][i] == routine_slot_flags[routine2][i + 1])
        model.add(routine_slot_flags[routine1][-1] == 0)
        model.add(routine_slot_flags[routine2][0] == 0)

    # Intermission is not in the first or last slot.
    model.add(routine_slot_flags["[Intermission]"][0] == 0)
    model.add(routine_slot_flags["[Intermission]"][-1] == 0)

    if True:
        # Place PT Contemp as the number before intermission.
        require_precedes("PT Contemp", "[Intermission]")

        # Place PT Jazz as the last number of the show.
        model.add(routine_slot_flags["PT Jazz"][-1] == 1)

        # Place senior as the number after intermission.
        require_precedes("[Intermission]", "Senior")

        # Place Alumni Jazz as the number before PT Contemp.
        require_precedes("Alumni Jazz", "PT Contemp")

        # Place Alumni Lyrical as the number before PT Jazz.
        require_precedes("Alumni Lyrical", "PT Jazz")

    ### Objective ###

    # Maximize the minimum wait time over all dancers.
    max_routine_count = max(dancer_routine_counts.values())
    min_wait_time_ub = (num_slots - max_routine_count) // (max_routine_count - 1)
    print(f"Greatest possible min_wait_time based on max_routine_count: {min_wait_time_ub}")
    if False:
        min_wait_time = model.new_int_var(1, min_wait_time_ub, "min_wait_time")
        for dancer, dancer_routines in dancer_wait_times.items():
            for routine, wait_time in dancer_routines.items():
                model.add(min_wait_time <= wait_time)
        model.maximize(min_wait_time)

    # Minimize how close the intermission is to the middle.
    middle_index = num_slots // 2
    intermission_to_middle_distance = sum(
        abs(i - middle_index) * routine_slot_flags["[Intermission]"][i] for i in range(num_slots)
    )
    max_intermission_dist = max(abs(i - middle_index) for i in range(num_slots))
    print(f"{max_intermission_dist=}")
    model.minimize(intermission_to_middle_distance)
    # model.add(intermission_to_middle_distance <= 2)

    # Minimize the number of dancer-routine wait times that are less than 2.
    num_short_waits = sum(
        2 - wait_time
        for dancer, dancer_routines in dancer_wait_times.items()
        for routine, wait_time in dancer_routines.items()
    )
    model.minimize(num_short_waits)
    # model.add(num_short_waits <= 3)

    model.minimize(num_short_waits * (max_intermission_dist + 1) + intermission_to_middle_distance)

    class MySolutionCallback(cp_model.CpSolverSolutionCallback):
        def __init__(self):
            super().__init__()

        def on_solution_callback(self):
            obj = int(self.objective_value)
            num_short_waits = obj // (max_intermission_dist + 1)
            intermission_to_middle_distance = obj % (max_intermission_dist + 1)

            print()
            routines_in_slot_by_slot = []
            for i in range(num_slots):
                routines_in_slot = [
                    routine for routine in routines.keys() if self.value(routine_slot_flags[routine][i])
                ]
                routines_in_slot_by_slot.append(routines_in_slot)
                print(f"    Slot {i:>2}:    {',  '.join(routines_in_slot)}")

                conflicts = defaultdict(list)
                for dancer in dancers:
                    if any((dancer in routines[r]) for r in routines_in_slot):
                        for j in range(max(0, i - 2), i):
                            for r in routines_in_slot_by_slot[j]:
                                if dancer in routines[r]:
                                    conflicts[dancer].append(r)
                for dancer, rs in conflicts.items():
                    print(f"        [{dancer}] is in {', '.join(f'[{r}]' for r in rs)}")
            print(f"^ Solution ({num_short_waits=}, {intermission_to_middle_distance=})")
            print()

    solver = cp_model.CpSolver()
    solver.parameters.log_search_progress = True
    solver.parameters.max_presolve_iterations = 10
    status = solver.solve(model, MySolutionCallback())

    is_optimal = status == cp_model.OPTIMAL
    print(f"OPTIMAL? {is_optimal}")
    print(f"FEASIBLE? {status == cp_model.FEASIBLE}")
    assert is_optimal

    for i in range(num_slots):
        routines_in_slot = [routine for routine in routines.keys() if solver.value(routine_slot_flags[routine][i])]
        print(f"Slot {i:>2}:    {',  '.join(routines_in_slot)}")

    for dancer, dancer_routines in dancer_wait_times.items():
        wait_times = {routine: solver.value(wait_time) for routine, wait_time in dancer_routines.items()}
        print(f"{dancer:>20}:  {wait_times}")
        # sum(w == 1 for w in wait_times)


if __name__ == "__main__":
    main()
