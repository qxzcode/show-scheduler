use indicatif::ProgressIterator;
use rand::seq::SliceRandom;

use crate::Routine;

struct ProblemInfo<'a> {
    routines: &'a [Routine],
    intermission_index: usize,
    intersection_counts: Vec<usize>,
}

impl<'a> ProblemInfo<'a> {
    pub fn new(routines: &'a [Routine]) -> Self {
        let n = routines.len();
        let mut intersection_counts = vec![0; n * n];
        for i in 0..n {
            for j in 0..n {
                let count = routines[i].dancers.intersection(&routines[j].dancers).count();
                intersection_counts[j + i * n] = count;
            }
        }

        Self {
            routines,
            intersection_counts,
            intermission_index: routines.iter().position(|r| r.name == "[Intermission]").unwrap(),
        }
    }

    pub fn intersection_count(&self, i: usize, j: usize) -> usize {
        let n = self.routines.len();
        debug_assert!(i != j);
        debug_assert!(i < n && j < n);
        self.intersection_counts[j + i * n]
    }
}

/// A candidate solution to the problem.
struct Solution<'a> {
    problem_info: &'a ProblemInfo<'a>,
    order: &'a mut [usize],
    intermission_index: usize,
    score: Score,
}

impl<'a> Solution<'a> {
    /// Creates a new solution initialized with the given `order`.
    pub fn new(problem_info: &'a ProblemInfo<'a>, order: &'a mut [usize]) -> Self {
        Self {
            intermission_index: order.iter().position(|&idx| idx == problem_info.intermission_index).unwrap(),
            score: score_order(problem_info, order),
            problem_info,
            order,
        }
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
        let get_num_dist_1 = |i1, i2| {
            if let (Some(r1), Some(r2)) = (self.order.get(i1 as usize), self.order.get(i2 as usize)) {
                self.problem_info.intersection_count(*r1, *r2)
            } else {
                0
            }
        };
        let old_num_dist_1 = get_num_dist_1(i - 1, i) + get_num_dist_1(j, j + 1);
        let new_num_dist_1 = get_num_dist_1(i - 1, j) + get_num_dist_1(i, j + 1);

        if new_num_dist_1 > old_num_dist_1 {
            // Can't be an improvement.
            return false;
        }

        // Check if the mutation decreases `num_dist_2`.
        let get_num_dist_2_triple = |i1, i2, i3| {
            let has_mid = self.order.get(i2 as usize).is_some_and(|r2| *r2 != self.problem_info.intermission_index);
            if has_mid { get_num_dist_1(i1, i3) } else { 0 }
        };
        let get_num_dist_2 = |i1, i2, i3, i4| get_num_dist_2_triple(i1, i2, i3) + get_num_dist_2_triple(i2, i3, i4);
        let old_num_dist_2 = get_num_dist_2(i - 2, i - 1, i, i + 1) + get_num_dist_2(j - 1, j, j + 1, j + 2);
        let new_num_dist_2 = get_num_dist_2(i - 2, i - 1, j, j - 1) + get_num_dist_2(i + 1, i, j + 1, j + 2);
        if new_num_dist_1 >= old_num_dist_1 && new_num_dist_2 > old_num_dist_2 {
            // Can't be an improvement.
            return false;
        }

        let i = i as usize;
        let j = j as usize;

        // Check if the mutation improves `intermission_middle_dist`.
        let old_intermission_middle_dist = self.score.2;
        let new_intermission_index = if (i..=j).contains(&self.intermission_index) {
            // Intermission is within the reversed segment.
            i + (j - self.intermission_index)
        } else {
            self.intermission_index
        };
        let new_intermission_middle_dist = get_middle_dist(n, new_intermission_index);

        if new_num_dist_1 >= old_num_dist_1
            && new_num_dist_2 >= old_num_dist_2
            && new_intermission_middle_dist >= old_intermission_middle_dist
        {
            // Not an improvement.
            return false;
        }

        // Perform the reversal and update metadata.
        self.order[i..=j].reverse();
        self.intermission_index = new_intermission_index;
        self.score = (
            self.score.0 + new_num_dist_1 - old_num_dist_1,
            self.score.1 + new_num_dist_2 - old_num_dist_2,
            new_intermission_middle_dist,
        );
        true
    }
}

pub fn optimize_order(routines: &[Routine]) -> (Vec<usize>, Score) {
    let n = routines.len();
    let mut rng = rand::rng();
    let problem_info = ProblemInfo::new(routines);

    let mut best_order = (0..n).collect::<Vec<_>>();
    let mut best_score = score_order(&problem_info, &best_order);

    let mut tmp_order = best_order.clone();
    for _ in (0..100_000).progress() {
        let score = hill_climb_order(&problem_info, &mut rng, &mut tmp_order);
        if score < best_score {
            best_score = score;
            best_order.clone_from(&tmp_order);
            println!("New best score: {:?}", best_score);
        }
    }
    (best_order, best_score)
}

fn hill_climb_order(problem_info: &ProblemInfo, rng: &mut impl rand::Rng, order: &mut [usize]) -> Score {
    let n = problem_info.routines.len();
    order.shuffle(rng);
    let mut solution = Solution::new(problem_info, order);
    let mut last_improvement = (n - 2, n - 1);
    'main_loop: loop {
        for i in 0..n {
            for j in (i + 1)..n {
                if (i, j) == last_improvement {
                    break 'main_loop;
                }

                if solution.reverse_if_improvement(i, j) {
                    // println!("Improved score to {:?} by reversing segment [{i}, {j}]", solution.score);

                    debug_assert_eq!(score_order(problem_info, solution.order), solution.score);

                    if solution.score == (0, 0, 0) {
                        println!("Found optimal score!");
                        return solution.score;
                    }
                    last_improvement = (i, j);
                }
            }
        }
    }
    // println!();

    solution.score
}

type Score = (usize, usize, usize); // (num_dist_1, num_dist_2, intermission_middle_dist)

fn score_order(problem_info: &ProblemInfo, order: &[usize]) -> Score {
    let mut num_dist_1 = 0;
    let mut num_dist_2 = 0;
    let n = order.len();
    for i in 0..(n - 1) {
        num_dist_1 += problem_info.intersection_count(order[i], order[i + 1]);
        if i + 2 < n && order[i + 1] != problem_info.intermission_index {
            num_dist_2 += problem_info.intersection_count(order[i], order[i + 2]);
        }
    }

    let intermission_index = order.iter().position(|&idx| idx == problem_info.intermission_index).unwrap();
    let intermission_middle_dist = get_middle_dist(n, intermission_index);

    (num_dist_1, num_dist_2, intermission_middle_dist)
}

fn get_middle_dist(n: usize, index: usize) -> usize {
    if index >= n / 2 {
        index - (n / 2)
    } else {
        let middle_index = if n.is_multiple_of(2) { (n / 2) - 1 } else { n / 2 };
        middle_index - index
    }
}
