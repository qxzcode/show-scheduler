use std::cmp::Ordering::{Greater, Less};

use crate::{Routine, preprocessing::ProblemInfo, randomize::Randomizer};

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub struct Score {
    pub num_dist_1: usize,
    pub num_dist_2: usize,
    pub intermission_middle_dist: usize,
}

impl Score {
    pub const PERFECT: Self = Self { num_dist_1: 0, num_dist_2: 0, intermission_middle_dist: 0 };
}

/// A candidate solution to the problem.
struct Solution<'a> {
    problem_info: &'a ProblemInfo,
    order: Box<[usize]>,
    intermission_index: usize,
    score: Score,
}

impl<'a> Solution<'a> {
    /// Creates a new solution initialized with a random order.
    pub fn new(problem_info: &'a ProblemInfo, randomizer: &mut Randomizer<impl rand::Rng>) -> Self {
        let mut order: Box<[usize]> = std::iter::repeat_n(0, problem_info.num_slots()).collect();
        randomizer.randomize_order(&mut order);
        let mut solution = Self { problem_info, order, intermission_index: 0, score: Score::PERFECT };
        solution.randomize(randomizer);
        solution
    }

    /// Randomizes the solution (and updates the intermission index and score accordingly).
    pub fn randomize(&mut self, randomizer: &mut Randomizer<impl rand::Rng>) {
        randomizer.randomize_order(&mut self.order);
        self.intermission_index = self.problem_info.intermission_index_in_order(&self.order);
        self.score = score_order(self.problem_info, &self.order);
    }

    /// Reverses the segment between indices `i` and `j` (inclusive) if it improves the score.
    /// Returns `true` if the reversal was performed and the score improved, `false` otherwise.
    pub fn reverse_if_improvement(&mut self, i: usize, j: usize) -> bool {
        // The reversal would convert this path:
        //     ..., i-2, i-1, i, i+1, ..., j-1, j, j+1, j+2, ...
        // into this path:
        //     ..., i-2, i-1, j, j-1, ..., i+1, i, j+1, j+2, ...

        let n = self.order.len();
        debug_assert!(i < j && j < n);

        let i = i as isize;
        let j = j as isize;

        // Check if the mutation decreases `num_dist_1`.
        let get_num_dist_1 = |i1: isize, i2: isize| {
            if let (Some(r1), Some(r2)) = (self.order.get(i1 as usize), self.order.get(i2 as usize)) {
                self.problem_info.intersection_count(*r1, *r2)
            } else {
                0
            }
        };
        let old_num_dist_1 = get_num_dist_1(i - 1, i) + get_num_dist_1(j, j + 1);
        let new_num_dist_1 = get_num_dist_1(i - 1, j) + get_num_dist_1(i, j + 1);

        let mut ord = new_num_dist_1.cmp(&old_num_dist_1);
        if ord == Greater {
            return false;
        }

        // Check if the mutation decreases `num_dist_2`.
        let get_num_dist_2_triple = |i1: isize, i2: isize, i3: isize| {
            let has_mid = self.order.get(i2 as usize).is_some_and(|r2| *r2 != self.problem_info.intermission_index());
            if has_mid { get_num_dist_1(i1, i3) } else { 0 }
        };
        let get_num_dist_2 = |i1: isize, i2: isize, i3: isize, i4: isize| {
            get_num_dist_2_triple(i1, i2, i3) + get_num_dist_2_triple(i2, i3, i4)
        };
        let old_num_dist_2 = get_num_dist_2(i - 2, i - 1, i, i + 1) + get_num_dist_2(j - 1, j, j + 1, j + 2);
        let new_num_dist_2 = get_num_dist_2(i - 2, i - 1, j, j - 1) + get_num_dist_2(i + 1, i, j + 1, j + 2);

        ord = ord.then(new_num_dist_2.cmp(&old_num_dist_2));
        if ord == Greater {
            return false;
        }

        let i = i as usize;
        let j = j as usize;

        // Check if the mutation improves `intermission_middle_dist`.
        let new_intermission_index = if (i..=j).contains(&self.intermission_index) {
            // Intermission is within the reversed segment.
            i + (j - self.intermission_index)
        } else {
            self.intermission_index
        };
        let new_intermission_middle_dist = get_middle_dist(n, new_intermission_index);

        ord = ord.then(new_intermission_middle_dist.cmp(&self.score.intermission_middle_dist));
        if ord != Less {
            return false;
        }

        // Perform the reversal and update metadata.
        self.order[i..=j].reverse();
        self.intermission_index = new_intermission_index;
        self.score = Score {
            num_dist_1: self.score.num_dist_1 + new_num_dist_1 - old_num_dist_1,
            num_dist_2: self.score.num_dist_2 + new_num_dist_2 - old_num_dist_2,
            intermission_middle_dist: new_intermission_middle_dist,
        };
        true
    }
}

pub fn optimize_order(routines: &[Routine], num_slots: usize, num_iterations: u32) -> (Box<[usize]>, Score) {
    let problem_info = ProblemInfo::new(routines, num_slots);

    let mut randomizer = Randomizer::new(&problem_info, rand::rng());

    let mut solution = Solution::new(&problem_info, &mut randomizer);
    let mut best_order = solution.order.clone();
    let mut best_score = solution.score;

    for _ in 0..num_iterations {
        solution.randomize(&mut randomizer);
        let score = hill_climb_order(&problem_info, &mut solution);
        if score < best_score {
            best_score = score;
            best_order.clone_from(&solution.order);
        }
    }

    (best_order, best_score)
}

/// Runs iterations indefinitely, calling `on_improvement` each time a better solution is found.
/// Stops when no improvement has been found for `NO_IMPROVEMENT_TIMEOUT_MS` milliseconds,
/// or immediately when the optimal score `(0, 0, 0)` is reached.
pub fn optimize_order_streaming<F: FnMut(&[usize], Score)>(
    routines: &[Routine],
    num_slots: usize,
    mut on_improvement: F,
) {
    const NO_IMPROVEMENT_TIMEOUT_MS: f64 = 5_000.0;

    let problem_info = ProblemInfo::new(routines, num_slots);
    let mut randomizer = Randomizer::new(&problem_info, rand::rng());

    let mut solution = Solution::new(&problem_info, &mut randomizer);
    // Hill-climb the initial random solution before reporting anything.
    let mut best_score = hill_climb_order(&problem_info, &mut solution);
    let mut best_order = solution.order.clone();
    on_improvement(&best_order, best_score);

    if best_score == Score::PERFECT {
        return;
    }

    let mut last_improvement_ms = js_sys::Date::now();

    loop {
        solution.randomize(&mut randomizer);
        let score = hill_climb_order(&problem_info, &mut solution);

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
                    debug_assert_eq!(score_order(problem_info, &solution.order), solution.score);

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

fn score_order(problem_info: &ProblemInfo, order: &[usize]) -> Score {
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

    Score { num_dist_1, num_dist_2, intermission_middle_dist }
}

fn get_middle_dist(n: usize, index: usize) -> usize {
    if index >= n / 2 {
        index - (n / 2)
    } else {
        let middle_index = if n.is_multiple_of(2) { (n / 2) - 1 } else { n / 2 };
        middle_index - index
    }
}
