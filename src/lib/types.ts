export interface InSlotConstraint {
    id: string;
    kind: 'in_slot';
    routine: string;
    slot: number;
}

export interface DirectlyBeforeConstraint {
    id: string;
    kind: 'directly_before';
    /** The routine that comes first (directly before `afterRoutine`). */
    beforeRoutine: string;
    /** The routine that comes immediately after `beforeRoutine`. */
    afterRoutine: string;
}

export type CustomConstraint = InSlotConstraint | DirectlyBeforeConstraint;
