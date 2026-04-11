import difflib
import itertools
import math
from collections import defaultdict

import pandas as pd
from ortools.sat.python import cp_model


def load_data() -> tuple[dict[str, list[str]], list[str], list[tuple[str, list[str]]]]:
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
    max_routine_name_len = max(len(routine) for routine in routines.keys())
    for routine in sorted(routines.keys(), key=lambda routine: (-len(routines[routine]), routine)):
        print(f"    {routine:<{max_routine_name_len}}  with {len(routines[routine])} dancer(s)")
    print()

    print(f"{len(dancers)} dancers:")
    dancer_routine_counts = {dancer: sum((dancer in dancers) for dancers in routines.values()) for dancer in dancers}
    max_dancer_name_len = max(len(dancer) for dancer in dancers)
    for dancer in sorted(dancers, key=lambda dancer: (-dancer_routine_counts[dancer], dancer)):
        print(f"    {dancer:<{max_dancer_name_len}}  in {dancer_routine_counts[dancer]} routine(s)")
    print()

    print("Possible duplicate dancer names:")
    for d1, close_matches in possible_dupes:
        print(f"    {d1}: {', '.join(close_matches)}")
    print()

    # CP-SAT
    print("Setting up CP-SAT model...")
    print()

    num_slots = 31 + 1  # +1 for intermission

    model = cp_model.CpModel()

    ### Variables and constraints ###

    # Possible slot contents.
    type Slot = tuple[str, str] | tuple[str]
    small_routines = [routine for routine, dancers_in_routine in routines.items() if 1 <= len(dancers_in_routine) <= 2]
    possible_slots: list[Slot] = [(routine,) for routine in routines.keys()] + [
        (r1, r2)
        for r1, r2 in itertools.combinations(small_routines, 2)
        if set(routines[r1]).isdisjoint(set(routines[r2]))
    ]
    print(len(possible_slots), "possible slot contents")
    print()

    dancers_in_slot = {slot: set(dancer for routine in slot for dancer in routines[routine]) for slot in possible_slots}

    non_initial_nodes = [
        (slot1, slot2)
        for slot1, slot2 in itertools.product(possible_slots, repeat=2)
        if dancers_in_slot[slot1].isdisjoint(dancers_in_slot[slot2])
    ]
    print(len(non_initial_nodes), "non-initial nodes")

    initial_nodes = [((), slot) for slot in possible_slots if slot != ("[Intermission]",)]
    print(len(initial_nodes), "initial nodes")

    type Node = tuple[Slot | tuple[()], Slot]
    nodes: list[Node] = non_initial_nodes + initial_nodes
    print(len(nodes), "total nodes")
    print()

    arc_flags: dict[tuple[Node | tuple[()], Node | tuple[()]], cp_model.IntVar] = {}

    # Add an arc from the start to each initial node.
    for node in initial_nodes:
        arc_flags[(), node] = model.new_bool_var(f"arc(start, {node})")

    # An arc from each possible slot content to each other feasible slot content.
    for node1 in nodes:
        for slot in possible_slots:
            node2 = (node1[1], slot)
            if slot not in node1 and dancers_in_slot[node1[1]].isdisjoint(dancers_in_slot[slot]):
                assert node2 in nodes
                arc_flags[node1, node2] = model.new_bool_var(f"arc({node1}, {node2})")

    # An arc from each non-initial node to the end.
    for node in non_initial_nodes:
        if node[1] != ("[Intermission]",):
            arc_flags[node, ()] = model.new_bool_var(f"arc({node}, end)")

    print(len(arc_flags), "arcs")
    print()

    is_node_used = {node: model.new_bool_var(f"is_node_used({node})") for node in nodes}
    is_before_intermission = {node: model.new_bool_var(f"is_before_intermission({node})") for node in nodes}

    for node in nodes:
        model.add_implication(is_before_intermission[node], is_node_used[node])

    out_arcs: dict[Node | tuple[()], list[tuple[Node | tuple[()], cp_model.IntVar]]] = defaultdict(list)
    for (node1, node2), arc_flag in arc_flags.items():
        out_arcs[node1].append((node2, arc_flag))

        node1_ibi = 1 if node1 == () else is_before_intermission[node1]
        node2_ibi = 0 if node2 == () else is_before_intermission[node2]
        if node2 != () and node2[1] == ("[Intermission]",):
            # Intermission itself is not before intermission.
            model.add(node2_ibi == 0)
        else:
            # Otherwise, before intermission propagates along arcs.
            model.add(node1_ibi == node2_ibi).only_enforce_if(arc_flag)

    # Circuit constraint to enforce that the solution consists of a single valid path.
    node_to_index = defaultdict(lambda: len(node_to_index))
    arcs = [(node_to_index[node1], node_to_index[node2], arc_flag) for (node1, node2), arc_flag in arc_flags.items()]
    arcs.extend((node_to_index[node], node_to_index[node], ~is_node_used[node]) for node in nodes)
    model.add_circuit(arcs)

    # Each routine appears exactly once.
    for routine in routines.keys():
        print(f"{routine:>20}: {len([used_flag for node, used_flag in is_node_used.items() if routine in node[1]])}")
        model.add_exactly_one(used_flag for node, used_flag in is_node_used.items() if routine in node[1])

    # Use exactly num_slots slots.
    model.add(sum(is_node_used.values()) == num_slots)

    ### "Extra" constraints ###

    def require_precedes(routine1: str, routine2: str):
        """Require that `routine1` immediately precedes `routine2` in the show order."""
        model.add(is_node_used[((routine1,), (routine2,))] == 1)

    def require_is_last(routine: str):
        """Require that `routine` is the last routine in the show order."""
        model.add_exactly_one(
            arc_flag
            for (node1, node2), arc_flag in arc_flags.items()
            if node1 != () and routine in node1[1] and node2 == ()
        )

    if True:
        # Place PT Contemp as the number before intermission.
        require_precedes("PT Contemp", "[Intermission]")

        # Place PT Jazz as the last number of the show.
        require_is_last("PT Jazz")

        # # Place senior as the number after intermission.
        # require_precedes("[Intermission]", "Senior")

        # Place Alumni Jazz as the number before PT Contemp.
        require_precedes("Alumni Jazz", "PT Contemp")

        # # Place Alumni Lyrical as the number before PT Jazz.
        # require_precedes("Alumni Lyrical", "PT Jazz")

    ### Objective ###

    # Minimize the number of dancer-routine wait times that are less than 2.
    num_short_waits = 0
    for (node1, node2), arc_flag in arc_flags.items():
        if node1 != () and node2 != ():
            slot1 = node1[0]
            slot2 = node1[1]
            slot3 = node2[1]
            if slot1 != () and slot2 != ("[Intermission]",) and slot3 != ():
                short_waits_for_this_arc = len(dancers_in_slot[slot1] & dancers_in_slot[slot3])
                num_short_waits += short_waits_for_this_arc * arc_flag

    # Minimize the distance from intermission to the middle.
    intermission_to_middle_distance = model.new_int_var(0, (num_slots - 1) // 2, "intermission_to_middle_distance")
    intermission_index = sum(is_before_intermission.values())
    if num_slots % 2 == 0:
        model.add(intermission_to_middle_distance >= (num_slots // 2 - 1) - intermission_index)
        model.add(intermission_to_middle_distance >= intermission_index - (num_slots // 2))
    else:
        model.add(intermission_to_middle_distance >= (num_slots // 2) - intermission_index)
        model.add(intermission_to_middle_distance >= intermission_index - (num_slots // 2))

    model.minimize(num_short_waits * num_slots + intermission_to_middle_distance)

    ### Solve! ###

    print("Solving...")
    print()

    def decode_solution(solution: cp_model.CpSolver | cp_model.CpSolverSolutionCallback) -> list[list[str]]:
        routines_in_slot_by_slot = []
        node = ()
        for _ in range(num_slots):
            for node2, arc_flag in out_arcs[node]:
                if node2 == ():
                    continue
                if solution.value(arc_flag):
                    routines_in_slot_by_slot.append(list(node2[1]))
                    node = node2
                    break
            else:
                raise RuntimeError("No outgoing arc found!")
        return routines_in_slot_by_slot

    class MySolutionCallback(cp_model.CpSolverSolutionCallback):
        def __init__(self):
            super().__init__()

        def on_solution_callback(self):
            obj = int(self.objective_value)
            num_short_waits = obj // num_slots
            intermission_to_middle_distance = obj % num_slots
            # num_short_waits = obj
            # intermission_to_middle_distance = "N/A"

            print()
            routines_in_slot_by_slot = decode_solution(self)
            for i, routines_in_slot in enumerate(routines_in_slot_by_slot):
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
    # solver.parameters.max_presolve_iterations = 1
    status = solver.solve(model, MySolutionCallback())

    is_optimal = status == cp_model.OPTIMAL
    print(f"OPTIMAL? {is_optimal}")
    print(f"FEASIBLE? {status == cp_model.FEASIBLE}")
    assert is_optimal

    routines_in_slot_by_slot = decode_solution(solver)
    for i, routines_in_slot in enumerate(routines_in_slot_by_slot):
        print(f"Slot {i:>2}:    {',  '.join(routines_in_slot)}")

    # for dancer, dancer_routines in dancer_wait_times.items():
    #     wait_times = {routine: solver.value(wait_time) for routine, wait_time in dancer_routines.items()}
    #     print(f"{dancer:>20}:  {wait_times}")
    #     # sum(w == 1 for w in wait_times)


if __name__ == "__main__":
    main()
