use std::{collections::HashSet, iter};

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

        let intermission_routine_index = routines
            .iter()
            .position(|r| r.name == "[Intermission]")
            .expect("routines should include an entry named '[Intermission]'");
        let intermission_index = meta_routines
            .iter()
            .position(|mr| matches!(mr, MetaRoutine::Single(i) if *i == intermission_routine_index))
            .expect("the intermission routine should be included as a Single meta-routine");

        let filter_single_mr_indices = |doubleable: bool| {
            meta_routines
                .iter()
                .enumerate()
                .filter(|(_, mr)| {
                    matches!(mr, MetaRoutine::Single(routine_index) if routines[*routine_index].is_doubleable() == doubleable)
                })
                .map(|(i, _)| i)
                .collect()
        };

        let non_doubleable_mr_indices: Box<[usize]> = filter_single_mr_indices(false);
        let doubleable_mr_indices = filter_single_mr_indices(true);

        // Assert that the doubleable indices are contiguous.
        assert!(doubleable_mr_indices.windows(2).all(|w| w[0] + 1 == w[1]));

        let doubleable_r_indices = doubleable_mr_indices.iter().map(|&mr_index| match meta_routines[mr_index] {
            MetaRoutine::Single(routine_index) => routine_index,
            _ => unreachable!("doubleable_mr_indices should only contain Single meta-routines"),
        });
        let mrs: &[MetaRoutine] = meta_routines.as_ref();
        let combined_mr_indices: Box<[Option<usize>]> = doubleable_r_indices
            .clone()
            .flat_map(|r1| {
                doubleable_r_indices.clone().map(move |r2| {
                    (r1 != r2).then(|| {
                        mrs.iter()
                            .position(|mr| {
                                matches!(mr, MetaRoutine::Double(i, j)
                                    if (*i == r1 && *j == r2) || (*i == r2 && *j == r1))
                            })
                            .expect("there should be a Double meta-routine for every pair of doubleable routines")
                    })
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
        let first_dmr_index = *self.doubleable_mr_indices.first().unwrap_or(&self.meta_routines.len());
        let i = mr1_index.checked_sub(first_dmr_index)?;
        let j = mr2_index.checked_sub(first_dmr_index)?;
        let double_mr_index = *self.combined_mr_indices.get(i * self.doubleable_mr_indices.len() + j)?.as_ref()?;
        debug_assert_eq!(
            double_mr_index,
            self.meta_routines().iter().position(|mr| {
                matches!(mr, MetaRoutine::Double(i, j) if (*i == mr1_index && *j == mr2_index) || (*i == mr2_index && *j == mr1_index))
            }).expect("there should be a Double meta-routine for every pair of doubleable routines"),
            "double_mr_index should match the position of the corresponding Double meta-routine"
        );
        Some(double_mr_index)
    }

    /// Returns the index of the intermission meta-routine (in the list returned by `meta_routines()`).
    pub fn intermission_index(&self) -> usize {
        self.intermission_index
    }

    /// Returns the index of the intermission meta-routine in the given order of meta-routines.
    pub fn intermission_index_in_order(&self, order: &[usize]) -> usize {
        order
            .iter()
            .position(|&idx| idx == self.intermission_index)
            .expect("the order should always include the intermission index")
    }

    /// Returns the number of dancers performing in both meta-routine `i` and meta-routine `j`.
    pub fn intersection_count(&self, i: usize, j: usize) -> usize {
        let n = self.meta_routines.len();
        debug_assert!(i < n && j < n);
        self.intersection_counts[j + i * n]
    }
}
