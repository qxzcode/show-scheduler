use rand::RngExt;

use crate::preprocessing::ProblemInfo;

/// Helper struct for sampling from a collection without replacement.
struct SamplerWithoutReplacement {
    collection: Box<[usize]>,
    num_remaining: usize,
}

impl FromIterator<usize> for SamplerWithoutReplacement {
    fn from_iter<T: IntoIterator<Item = usize>>(iter: T) -> Self {
        let collection: Box<[usize]> = iter.into_iter().collect();
        let num_remaining = collection.len();
        Self { collection, num_remaining }
    }
}

impl SamplerWithoutReplacement {
    /// Selects a random element from the remaining elements in the collection,
    /// removes it from the collection, and returns it.
    ///
    /// Returns `None` if there are no remaining elements to draw.
    fn draw(&mut self, rng: &mut impl rand::Rng) -> Option<usize> {
        (self.num_remaining > 0).then(|| {
            let i = rng.random_range(0..self.num_remaining);
            let value = self.collection[i];
            self.num_remaining -= 1;
            self.collection.swap(i, self.num_remaining);
            value
        })
    }

    /// Reset the sampler so that all elements in the collection are available to be drawn again.
    fn reset(&mut self) {
        self.num_remaining = self.collection.len();
    }
}

pub struct Randomizer<'a, Rng> {
    problem_info: &'a ProblemInfo,
    rng: Rng,

    // Temporary variables used during randomization; cached to use fewer allocations.
    available_slots: SamplerWithoutReplacement,
    available_doubleable_mr_indices: SamplerWithoutReplacement,
}

impl<'a, Rng: rand::Rng> Randomizer<'a, Rng> {
    pub fn new(problem_info: &'a ProblemInfo, rng: Rng) -> Self {
        Self {
            problem_info,
            rng,
            available_slots: (0..problem_info.num_slots()).collect(),
            available_doubleable_mr_indices: problem_info.doubleable_mr_indices().iter().copied().collect(),
        }
    }

    pub fn randomize_order(&mut self, order: &mut [usize]) {
        assert_eq!(order.len(), self.problem_info.num_slots());

        self.available_slots.reset();

        // Place all non-doubleable meta-routines in random slots.
        for &mr in self.problem_info.non_doubleable_mr_indices() {
            order[self
                .available_slots
                .draw(&mut self.rng)
                .expect("there should be enough slots to place all routines")] = mr;
        }

        let num_double_slots = self.available_slots.num_remaining;
        self.available_doubleable_mr_indices.reset();

        // Place random doubleable meta-routines in random slots until all slots have a routine.
        while let Some(slot) = self.available_slots.draw(&mut self.rng) {
            let mr = self
                .available_doubleable_mr_indices
                .draw(&mut self.rng)
                .expect("there should be enough routines to fill all slots");
            order[slot] = mr;
        }

        // At this point, all slots are filled with one routine in each.
        // Now we need to place the remaining doubleable routines in slots that already have another doubleable routine.

        // Place the remaining doubleable meta-routines in random double slots.
        self.available_slots.num_remaining = num_double_slots; // Put the last `num_double_slots` slots back.
        while let Some(mr) = self.available_doubleable_mr_indices.draw(&mut self.rng) {
            let slot =
                self.available_slots.draw(&mut self.rng).expect("there should be enough slots to place all routines");
            order[slot] =
                self.problem_info.double_mr_index(order[slot], mr).expect("the two routines should be doubleable");
        }
    }
}
