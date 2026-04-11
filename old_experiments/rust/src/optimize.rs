use std::{collections::HashSet, iter};

use indicatif::ProgressIterator;
use rand::seq::SliceRandom;

use crate::Routine;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum MetaRoutine {
    Single(usize),
    Double(usize, usize),
}

impl MetaRoutine {
    /// Returns an iterator over the indices of the routines contained in this meta-routine.
    pub fn routine_indices(&self) -> impl Iterator<Item = usize> {
        match self {
            MetaRoutine::Single(i) => iter::once(*i).chain(None),
            MetaRoutine::Double(i, j) => iter::once(*i).chain(Some(*j)),
        }
    }

    /// Returns an iterator over all the dancers performing in this meta-routine.
    pub fn dancers<'a>(&self, routines: &'a [Routine]) -> impl Iterator<Item = &'a String> {
        self.routine_indices().flat_map(move |i| routines[i].dancers.iter())
    }

    /// Returns whether this meta-routine contains the routine with the given index.
    pub fn contains_routine(&self, routine_index: usize) -> bool {
        match self {
            MetaRoutine::Single(i) => *i == routine_index,
            MetaRoutine::Double(i, j) => *i == routine_index || *j == routine_index,
        }
    }
}

/// Pre-processed information about a particular problem instance.
#[derive(Clone, Debug)]
pub struct ProblemInfo {
    num_slots: usize,
    routines: Box<[Routine]>,
    meta_routines: Box<[MetaRoutine]>,
    non_doubleable_mr_indices: Box<[usize]>,
    doubleable_mr_indices: Box<[usize]>,
    combined_mr_indices: Box<[Option<usize>]>,
    intermission_index: usize,
    intersection_counts: Vec<usize>,
}

impl ProblemInfo {
    pub fn new(routines: &[Routine], num_slots: usize) -> Self {
        let mut meta_routines = Vec::new();
        meta_routines.extend((0..routines.len()).map(MetaRoutine::Single)); // All single routines.
        for i in 0..routines.len() {
            for j in (i + 1)..routines.len() {
                // Only allow "doubling-up" pairs of routines that:
                //  - Don't have any dancers in common.
                //  - Are individually eligible to be doubled up.
                if routines[i].dancers.is_disjoint(&routines[j].dancers)
                    && routines[i].is_doubleable()
                    && routines[j].is_doubleable()
                {
                    meta_routines.push(MetaRoutine::Double(i, j));
                }
            }
        }
        println!("Problem has {} routines and {} meta-routines.", routines.len(), meta_routines.len());

        let n = meta_routines.len();
        let mut intersection_counts = vec![0; n * n];
        for i in 0..n {
            let dancers_i: HashSet<_> = meta_routines[i].dancers(routines).collect();
            for j in 0..n {
                let dancers_j: HashSet<_> = meta_routines[j].dancers(routines).collect();
                let count = dancers_i.intersection(&dancers_j).count();
                intersection_counts[j + i * n] = count;
            }
        }

        let intermission_routine_index = routines.iter().position(|r| r.name == "[Intermission]").unwrap();
        let intermission_index = meta_routines
            .iter()
            .position(|mr| matches!(mr, MetaRoutine::Single(i) if *i == intermission_routine_index))
            .unwrap();

        let filter_single_mr_indices = |doubleable: bool| {
            meta_routines.iter().enumerate()
                .filter(|(_, mr)| matches!(mr, MetaRoutine::Single(routine_index) if routines[*routine_index].is_doubleable() == doubleable))
                .map(|(i, _)| i)
                .collect()
        };

        let non_doubleable_mr_indices: Box<[usize]> = filter_single_mr_indices(false);
        let doubleable_mr_indices = filter_single_mr_indices(true);

        // Assert that the doubleable indices are contiguous.
        assert!(doubleable_mr_indices.windows(2).all(|w| w[0] + 1 == w[1]));

        let doubleable_r_indices = doubleable_mr_indices.iter().map(|&mr_index| match meta_routines[mr_index] {
            MetaRoutine::Single(routine_index) => routine_index,
            _ => unreachable!(),
        });
        let mrs: &[MetaRoutine] = meta_routines.as_ref();
        let combined_mr_indices: Box<[Option<usize>]> = doubleable_r_indices.clone()
            .flat_map(|r1| {
                doubleable_r_indices.clone().map(move |r2| {
                    (r1 != r2).then(|| mrs.iter()
                        .position(|mr| matches!(mr, MetaRoutine::Double(i, j) if (*i == r1 && *j == r2) || (*i == r2 && *j == r1)))
                        .unwrap())
                })
            })
            .collect();

        Self {
            num_slots,
            routines: routines.into(),
            non_doubleable_mr_indices,
            doubleable_mr_indices,
            combined_mr_indices,
            meta_routines: meta_routines.into(),
            intersection_counts,
            intermission_index,
        }
    }

    /// Returns the number of time slots that the routines must be scheduled into (including the intermission).
    pub fn num_slots(&self) -> usize {
        self.num_slots
    }

    /// Returns the ordered list of routines in the problem.
    pub fn routines(&self) -> &[Routine] {
        self.routines.as_ref()
    }

    /// Returns the ordered list of meta-routines in the problem.
    pub fn meta_routines(&self) -> &[MetaRoutine] {
        self.meta_routines.as_ref()
    }

    /// Returns the indices of the meta-routines that contain a single routine that is not eligible to be doubled up.
    pub fn non_doubleable_mr_indices(&self) -> &[usize] {
        self.non_doubleable_mr_indices.as_ref()
    }

    /// Returns the indices of the meta-routines that contain a single routine that is eligible to be doubled up.
    pub fn doubleable_mr_indices(&self) -> &[usize] {
        self.doubleable_mr_indices.as_ref()
    }

    /// Returns the index of the meta-routine that results from combining the two given single-routine meta-routines.
    pub fn double_mr_index(&self, mr1_index: usize, mr2_index: usize) -> Option<usize> {
        let first_dmr_index = *self.doubleable_mr_indices.first().unwrap();
        let i = mr1_index.checked_sub(first_dmr_index)?;
        let j = mr2_index.checked_sub(first_dmr_index)?;
        let double_mr_index = *self.combined_mr_indices.get(i * self.doubleable_mr_indices.len() + j)?.as_ref()?;
        debug_assert_eq!(
            double_mr_index,
            self.meta_routines().iter().position(|mr| {
                matches!(mr, MetaRoutine::Double(i, j) if (*i == mr1_index && *j == mr2_index) || (*i == mr2_index && *j == mr1_index))
            }).unwrap(),
        );
        Some(double_mr_index)
    }

    /// Returns the index of the intermission meta-routine (in the list returned by `meta_routines()`).
    pub fn intermission_index(&self) -> usize {
        self.intermission_index
    }

    /// Returns the number of dancers performing in both meta-routine `i` and meta-routine `j`.
    pub fn intersection_count(&self, i: usize, j: usize) -> usize {
        let n = self.meta_routines.len();
        debug_assert!(i < n && j < n);
        self.intersection_counts[j + i * n]
    }
}

/// A candidate solution to the problem.
struct Solution<'a> {
    problem_info: &'a ProblemInfo,
    order: &'a mut [usize],
    intermission_index: usize,
    score: Score,
}

impl<'a> Solution<'a> {
    /// Creates a new solution initialized with the given `order`.
    pub fn new(problem_info: &'a ProblemInfo, order: &'a mut [usize]) -> Self {
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
    let problem_info = ProblemInfo::new(routines, n);

    let mut best_order = (0..n).collect::<Vec<_>>();
    let mut best_score = score_order(&problem_info, &best_order);

    let mut tmp_order = best_order.clone();
    for _ in (0..100_000).progress() {
        tmp_order.shuffle(&mut rng);
        let score = hill_climb_order(&problem_info, &mut tmp_order);
        if score < best_score {
            best_score = score;
            best_order.clone_from(&tmp_order);
            println!("New best score: {:?}", best_score);
        }
    }
    (best_order, best_score)
}

pub fn hill_climb_order(problem_info: &ProblemInfo, order: &mut [usize]) -> Score {
    let n = problem_info.routines.len();
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
