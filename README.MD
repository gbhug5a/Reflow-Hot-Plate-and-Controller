Note:  There is a video related to the hot plate:

[https://www.youtube.com/watch?v=hqwbDlXcs_I](https://www.youtube.com/watch?v=hqwbDlXcs_I)


## Reflow-Hot-Plate-and-Controller

This repository describes a simple and inexpensive reflow device consisting
of a $10 Walmart hot plate, a $5 circular saw blade from Home Depot, and
three machine screws from Ace Hardware.  It requires no controller.  The
operator simply turns the power on, then off, at two specific points in the
reflow process.  A full description is found in the Hotplate folder in the
file:

Reflow-Hot-Plate.pdf

Also included are a schematic and Arduino sketch for a duty-cycle reflow
controller that does not use any temperature feedback, but rather executes a
set sequence of duty-cycle stages to produce the desired temperature profile.
The controller was originally designed to support the reflow hot plate, but
in the end it was found that no controller was needed for that device.
Information is included here in case it might be of use to someone for
another hot plate or for a toaster oven.  A full description can be found in
the Controller folder in the file:

Duty-Cycle-Reflow-Controller.pdf
