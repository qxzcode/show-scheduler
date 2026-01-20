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
    let mut best_score = score_order(problem_info, order);
    let mut last_improvement = (n - 1, n - 1);
    'main_loop: loop {
        for i in 0..n {
            for j in (i + 1)..n {
                if (i, j) == last_improvement {
                    break 'main_loop;
                }

                order.swap(i, j);
                let current_score = score_order(problem_info, order);
                if current_score < best_score {
                    best_score = current_score;
                    // println!(
                    //     "Improved score to {:?} by swapping {} and {}",
                    //     best_score, problem_info.routines[order[i]].name, problem_info.routines[order[j]].name
                    // );
                    if best_score == (0, 0, 0) {
                        println!("Found optimal score!");
                        return best_score;
                    }
                    last_improvement = (i, j);
                } else {
                    order.swap(i, j); // revert
                }
            }
        }
    }
    // println!();

    best_score
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

    let intermission_middle_dist = if intermission_index >= n / 2 {
        intermission_index - (n / 2)
    } else {
        let middle_index = if n.is_multiple_of(2) { (n / 2) - 1 } else { n / 2 };
        middle_index - intermission_index
    };

    (num_dist_1, num_dist_2, intermission_middle_dist)
}
