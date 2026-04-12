use std::cmp::Ordering::{Greater, Less};

use crate::{
    preprocessing::{CompiledConstraint, ProblemInfo},
    randomize::Randomizer,
};

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub struct Score {
    // constraint_violations is listed first so it is the most significant field in the
    // lexicographic Ord derive — the optimizer treats satisfying constraints as the top
    // priority, above all performer-conflict objectives.
    pub constraint_violations: usize,
    pub num_dist_1: usize,
    pub clamped_intermission_mid: usize, // max(0, intermission_middle_dist - intermission_tolerance)
    pub num_dist_2: usize,
    pub intermission_middle_dist: usize,
}

impl Score {
    pub const PERFECT: Self = Self {
        constraint_violations: 0,
        num_dist_1: 0,
        clamped_intermission_mid: 0,
        num_dist_2: 0,
        intermission_middle_dist: 0,
    };
}

/// A candidate solution to the problem.
struct Solution<'a> {
    problem_info: &'a ProblemInfo,
    order: Box<[usize]>,
    intermission_index: usize,
    score: Score,
    /// Inverse of `order`: `pos_of_mr[mr_idx]` is the position of meta-routine `mr_idx`
    /// in the current order. Initialised to `usize::MAX` for meta-routines not in the
    /// current order, but all meta-routines referenced by compiled constraints are always
    /// present in the order — callers must never query an absent entry.
    pos_of_mr: Box<[usize]>,
    /// `active_mr_for_routine[routine_idx]` is the meta-routine index that currently
    /// contains `routine_idx`. Rebuilt on each `randomize` call; stable during hill-climbing
    /// (reversals only reorder meta-routines, never reassign routines between them).
    active_mr_for_routine: Box<[usize]>,
}

impl<'a> Solution<'a> {
    /// Creates a new solution initialized with a random order.
    pub fn new(problem_info: &'a ProblemInfo, randomizer: &mut Randomizer<impl rand::Rng>) -> Self {
        let num_mrs = problem_info.meta_routines().len();
        let num_routines = problem_info.routines().len();
        let mut order: Box<[usize]> = std::iter::repeat_n(0, problem_info.num_slots()).collect();
        randomizer.randomize_order(&mut order);
        let mut solution = Self {
            problem_info,
            order,
            intermission_index: 0,
            score: Score::PERFECT,
            pos_of_mr: vec![usize::MAX; num_mrs].into_boxed_slice(),
            active_mr_for_routine: vec![usize::MAX; num_routines].into_boxed_slice(),
        };
        solution.randomize(randomizer);
        solution
    }

    /// Randomizes the solution (and rebuilds all derived state accordingly).
    pub fn randomize(&mut self, randomizer: &mut Randomizer<impl rand::Rng>) {
        randomizer.randomize_order(&mut self.order);
        self.intermission_index = self.problem_info.intermission_index_in_order(&self.order);

        // Rebuild pos_of_mr and active_mr_for_routine from scratch.
        self.pos_of_mr.fill(usize::MAX);
        self.active_mr_for_routine.fill(usize::MAX);
        for (pos, &mr_idx) in self.order.iter().enumerate() {
            self.pos_of_mr[mr_idx] = pos;
            for routine_idx in self.problem_info.meta_routines()[mr_idx].routine_indices() {
                self.active_mr_for_routine[routine_idx] = mr_idx;
            }
        }

        self.score = score_order(self.problem_info, &self.order, &self.pos_of_mr, &self.active_mr_for_routine);
    }

    /// Reverses the segment between indices `i` and `j` (inclusive) if it strictly improves
    /// the score. Returns `true` if the reversal was performed, `false` otherwise.
    pub fn reverse_if_improvement(&mut self, i: usize, j: usize) -> bool {
        // The reversal converts:
        //     ..., i-2, i-1, i, i+1, ..., j-1, j, j+1, j+2, ...
        // into:
        //     ..., i-2, i-1, j, j-1, ..., i+1, i, j+1, j+2, ...

        let n = self.order.len();
        debug_assert!(i < j && j < n);

        let is = i as isize;
        let js = j as isize;

        // ── Constraint violations (highest priority) ─────────────────────────
        let new_constraint_violations = self
            .problem_info
            .constraints()
            .iter()
            .filter(|c| !constraint_satisfied_after_reversal(c, &self.pos_of_mr, &self.active_mr_for_routine, i, j))
            .count();
        let mut ord = new_constraint_violations.cmp(&self.score.constraint_violations);
        if ord == Greater {
            return false;
        }

        // ── num_dist_1 ────────────────────────────────────────────────────────
        let get_num_dist_1 = |i1: isize, i2: isize| {
            if let (Some(r1), Some(r2)) = (self.order.get(i1 as usize), self.order.get(i2 as usize)) {
                self.problem_info.intersection_count(*r1, *r2)
            } else {
                0
            }
        };
        let old_num_dist_1 = get_num_dist_1(is - 1, is) + get_num_dist_1(js, js + 1);
        let new_num_dist_1 = get_num_dist_1(is - 1, js) + get_num_dist_1(is, js + 1);

        ord = ord.then(new_num_dist_1.cmp(&old_num_dist_1));
        if ord == Greater {
            return false;
        }

        // ── clamped_intermission_mid ──────────────────────────────────────────
        let new_intermission_index = if (i..=j).contains(&self.intermission_index) {
            // Intermission is within the reversed segment.
            i + (j - self.intermission_index)
        } else {
            self.intermission_index
        };
        let new_intermission_middle_dist = get_middle_dist(n, new_intermission_index);
        let new_clamped_intermission_mid =
            new_intermission_middle_dist.saturating_sub(self.problem_info.intermission_tolerance());

        ord = ord.then(new_clamped_intermission_mid.cmp(&self.score.clamped_intermission_mid));
        if ord == Greater {
            return false;
        }

        // ── num_dist_2 ────────────────────────────────────────────────────────
        let get_num_dist_2_triple = |i1: isize, i2: isize, i3: isize| {
            let has_mid = self.order.get(i2 as usize).is_some_and(|r2| *r2 != self.problem_info.intermission_index());
            if has_mid { get_num_dist_1(i1, i3) } else { 0 }
        };
        let get_num_dist_2 = |i1: isize, i2: isize, i3: isize, i4: isize| {
            get_num_dist_2_triple(i1, i2, i3) + get_num_dist_2_triple(i2, i3, i4)
        };
        let old_num_dist_2 = get_num_dist_2(is - 2, is - 1, is, is + 1) + get_num_dist_2(js - 1, js, js + 1, js + 2);
        let new_num_dist_2 = get_num_dist_2(is - 2, is - 1, js, js - 1) + get_num_dist_2(is + 1, is, js + 1, js + 2);

        ord = ord.then(new_num_dist_2.cmp(&old_num_dist_2));
        if ord == Greater {
            return false;
        }

        // ── intermission_middle_dist (final tiebreaker) ───────────────────────
        ord = ord.then(new_intermission_middle_dist.cmp(&self.score.intermission_middle_dist));
        if ord != Less {
            return false;
        }

        // ── Apply the reversal ────────────────────────────────────────────────
        self.order[i..=j].reverse();
        self.intermission_index = new_intermission_index;

        // Update pos_of_mr for every position that moved.
        for pos in i..=j {
            self.pos_of_mr[self.order[pos]] = pos;
        }

        self.score = Score {
            constraint_violations: new_constraint_violations,
            num_dist_1: self.score.num_dist_1 + new_num_dist_1 - old_num_dist_1,
            clamped_intermission_mid: new_clamped_intermission_mid,
            num_dist_2: self.score.num_dist_2 + new_num_dist_2 - old_num_dist_2,
            intermission_middle_dist: new_intermission_middle_dist,
        };
        true
    }
}

/// Runs iterations indefinitely, calling `on_improvement` each time a better solution is found.
/// Stops when no improvement has been found for `NO_IMPROVEMENT_TIMEOUT_MS` milliseconds,
/// or immediately when an optimal score is reached.
pub fn optimize_order_streaming<F: FnMut(&[usize], Score)>(problem_info: &ProblemInfo, mut on_improvement: F) {
    const NO_IMPROVEMENT_TIMEOUT_MS: f64 = 15_000.0;

    let mut randomizer = Randomizer::new(problem_info, rand::rng());

    let mut solution = Solution::new(problem_info, &mut randomizer);
    // Hill-climb the initial random solution before reporting anything.
    let mut best_score = hill_climb_order(problem_info, &mut solution);
    let mut best_order = solution.order.clone();
    on_improvement(&best_order, best_score);

    if best_score == Score::PERFECT {
        return;
    }

    let mut last_improvement_ms = js_sys::Date::now();

    loop {
        solution.randomize(&mut randomizer);
        let score = hill_climb_order(problem_info, &mut solution);

        if score < best_score {
            best_score = score;
            best_order.clone_from(&solution.order);
            on_improvement(&best_order, best_score);
            last_improvement_ms = js_sys::Date::now();

            if best_score == Score::PERFECT {
                break;
            }
        }

        if js_sys::Date::now() - last_improvement_ms > NO_IMPROVEMENT_TIMEOUT_MS {
            break;
        }
    }
}

fn hill_climb_order(problem_info: &ProblemInfo, solution: &mut Solution) -> Score {
    let n = problem_info.num_slots();
    let mut last_improvement = (n - 2, n - 1);
    'main_loop: loop {
        for i in 0..n {
            for j in (i + 1)..n {
                if (i, j) == last_improvement {
                    break 'main_loop;
                }

                if solution.reverse_if_improvement(i, j) {
                    debug_assert_eq!(
                        score_order(
                            problem_info,
                            &solution.order,
                            &solution.pos_of_mr,
                            &solution.active_mr_for_routine
                        ),
                        solution.score
                    );

                    if solution.score == Score::PERFECT {
                        // Found an optimal solution; no need to continue climbing.
                        return solution.score;
                    }
                    last_improvement = (i, j);
                }
            }
        }
    }

    solution.score
}

fn score_order(
    problem_info: &ProblemInfo,
    order: &[usize],
    pos_of_mr: &[usize],
    active_mr_for_routine: &[usize],
) -> Score {
    #[cfg(debug_assertions)]
    {
        // Verify that each routine appears exactly once.
        let mut seen = vec![false; problem_info.routines().len()];
        for &idx in order {
            for routine_idx in problem_info.meta_routines()[idx].routine_indices() {
                debug_assert!(!seen[routine_idx], "Duplicate routine index in order: {routine_idx}");
                seen[routine_idx] = true;
            }
        }
        debug_assert!(seen.into_iter().all(|x| x), "Not all routines are included in the order");
    }

    let mut num_dist_1 = 0;
    let mut num_dist_2 = 0;
    let n = order.len();
    for i in 0..(n - 1) {
        num_dist_1 += problem_info.intersection_count(order[i], order[i + 1]);
        if i + 2 < n && order[i + 1] != problem_info.intermission_index() {
            num_dist_2 += problem_info.intersection_count(order[i], order[i + 2]);
        }
    }

    let intermission_index = problem_info.intermission_index_in_order(order);
    let intermission_middle_dist = get_middle_dist(n, intermission_index);
    let clamped_intermission_mid = intermission_middle_dist.saturating_sub(problem_info.intermission_tolerance());

    let constraint_violations = count_constraint_violations(problem_info, pos_of_mr, active_mr_for_routine);

    Score { constraint_violations, num_dist_1, clamped_intermission_mid, num_dist_2, intermission_middle_dist }
}

// ── Constraint helpers ─────────────────────────────────────────────────────

/// Counts how many compiled constraints are currently violated, given pre-built
/// `pos_of_mr` and `active_mr_for_routine` lookup tables.
fn count_constraint_violations(
    problem_info: &ProblemInfo,
    pos_of_mr: &[usize],
    active_mr_for_routine: &[usize],
) -> usize {
    problem_info.constraints().iter().filter(|c| !constraint_satisfied(c, pos_of_mr, active_mr_for_routine)).count()
}

/// Returns the new position of `pos` after reversing the segment `[i, j]`.
#[inline(always)]
fn reversed_pos(pos: usize, i: usize, j: usize) -> usize {
    if pos >= i && pos <= j { i + j - pos } else { pos }
}

/// Checks whether `constraint` is satisfied in the current solution state.
fn constraint_satisfied(constraint: &CompiledConstraint, pos_of_mr: &[usize], active_mr_for_routine: &[usize]) -> bool {
    match constraint {
        CompiledConstraint::InSlot { routine_idx, target_slot } => {
            let mr_idx = active_mr_for_routine[*routine_idx];
            debug_assert_ne!(mr_idx, usize::MAX, "InSlot constraint routine has no active meta-routine");
            let pos = pos_of_mr[mr_idx];
            debug_assert_ne!(pos, usize::MAX, "InSlot constraint meta-routine not found in pos_of_mr");
            pos == *target_slot
        }
        CompiledConstraint::DirectlyBefore { after_routine_idx, before_routine_idx } => {
            let after_mr = active_mr_for_routine[*after_routine_idx];
            let before_mr = active_mr_for_routine[*before_routine_idx];
            debug_assert_ne!(after_mr, usize::MAX, "DirectlyBefore after_routine has no active meta-routine");
            debug_assert_ne!(before_mr, usize::MAX, "DirectlyBefore before_routine has no active meta-routine");
            let pos_after = pos_of_mr[after_mr];
            let pos_before = pos_of_mr[before_mr];
            debug_assert_ne!(pos_after, usize::MAX, "DirectlyBefore after meta-routine not found in pos_of_mr");
            debug_assert_ne!(pos_before, usize::MAX, "DirectlyBefore before meta-routine not found in pos_of_mr");
            // Both pos values are valid positions (0..num_slots), so pos_before + 1 never
            // overflows — the maximum value is num_slots - 1 + 1 = num_slots.
            pos_before + 1 == pos_after
        }
    }
}

/// Checks whether `constraint` would be satisfied after reversing segment `[i, j]`,
/// without modifying any state.
fn constraint_satisfied_after_reversal(
    constraint: &CompiledConstraint,
    pos_of_mr: &[usize],
    active_mr_for_routine: &[usize],
    i: usize,
    j: usize,
) -> bool {
    match constraint {
        CompiledConstraint::InSlot { routine_idx, target_slot } => {
            let mr_idx = active_mr_for_routine[*routine_idx];
            debug_assert_ne!(mr_idx, usize::MAX);
            let pos = pos_of_mr[mr_idx];
            debug_assert_ne!(pos, usize::MAX);
            reversed_pos(pos, i, j) == *target_slot
        }
        CompiledConstraint::DirectlyBefore { after_routine_idx, before_routine_idx } => {
            let after_mr = active_mr_for_routine[*after_routine_idx];
            let before_mr = active_mr_for_routine[*before_routine_idx];
            debug_assert_ne!(after_mr, usize::MAX);
            debug_assert_ne!(before_mr, usize::MAX);
            let pos_after = pos_of_mr[after_mr];
            let pos_before = pos_of_mr[before_mr];
            debug_assert_ne!(pos_after, usize::MAX);
            debug_assert_ne!(pos_before, usize::MAX);
            reversed_pos(pos_before, i, j) + 1 == reversed_pos(pos_after, i, j)
        }
    }
}

fn get_middle_dist(n: usize, index: usize) -> usize {
    if index >= n / 2 {
        index - (n / 2)
    } else {
        let middle_index = if n.is_multiple_of(2) { (n / 2) - 1 } else { n / 2 };
        middle_index - index
    }
}
