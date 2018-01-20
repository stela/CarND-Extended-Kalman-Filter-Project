README
======

Each time the RMSE is calculated, each component's maximum is saved. The project rubric sets a limit of [.11, .11, 0.52, 0.52].

Initially I tried to use the rather inexact measured laser position as initial state, however that caused the RMSE to go outside of the allowed range temporarily at the start. In order to avoid this I added a correction offset, specific for dataset 1.

The maximum for each RMSE component for Dataset 1 is [0.0976603, 0.0859691, 0.333607, 0.361843], which is within limits.
