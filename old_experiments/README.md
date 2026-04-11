# Show scheduler

This is a tool that helps find a good **order for the acts/performances in a show** that
- minimizes the number of performers who have a short turnaround between acts to change costume,
- while also respecting certain desired constraints on when certain acts should perform.

It sets up the discrete optimization problem and uses Google's OR-Tools to find solutions.
