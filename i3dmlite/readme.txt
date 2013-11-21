Building:
On mac, run make; on windows, build with visual studio.  I've attempted to include dependencies as needed, so ideally it should just work.

Running:
To run, pass a wavefront obj file (a polygonal mesh) to the i3dm program.  On windows you can just drag and drop the file onto the executable.  On mac or if you prefer the command line, the file should be the only argument to the program (so, "./i3dm nameofyourfile.obj")

When the program runs, you should see your obj file's shape and a "tweak bar" that lets you rotate that object, zoom, and provides fitting options.

To get started, draw strokes on the part of the object you would like to fit with a primitive, and select the desired primitive type by choosing the "Fitting Mode" on the tweak bar.  Then click "Fit Primitive".  A sweep profile will appear on the right side of the screen, which you can then click-and-drag to edit.  Right click to remove parts of the profile curve; this will de-select them so they will no longer be treated as part of the stationary sweep.

Detailed usage guide:

Here's a full list of the program menu options, and what each option does:

- Clear Strokes: This clears strokes you've drawn previously.
- RMB mode: If you've right-clicked to remove too much of a profile curve, you can toggle this to "Enables" and then right-click to re-add previously removed parts of the sweep.
- Helical motions: Lets the stationary sweep fitter find helical motions.  By default, it only fits surfaces of revolution.
- Show sweep paths: Show a simple visulation of the sweep motion found by the stationary sweep fitter.
- Fit iterations: Controls the number of fitting iterations performed by the sweep fitter.
- Segment filter size: Controls a post-process filter used to essentially de-noise the results, so if tiny single-triangle patches are not included in your segmentation, those will be added back by the filter.
- Edit radius (2D): Controls how large of a region your 2D profile curve edits will apply to.

Known issues / release notes:

Currently the segmentation is a bit overly conservative, and often requires extra strokes to fit the whole desired shape.

Planned future features:

 - Progressive sweep fitting and editing
 - Quadric fitting and editing
 - Freeform surfaces editing
 - Transformations between surface fits
 - Undo/redo button
 - Optional freeform "glue" to maintain continuity between edited surfaces and adjacent surfaces.