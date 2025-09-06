# nodeNetwork -custom tools

Tools

A compact suite of eight custom Grasshopper/Rhino scripts for digital-fabrication workflows (2D/3D geometry generation & joinery).

SubX — Sub-curves at intersections
What it does: Finds intersections in a curve network and emits short sub-curves from each node for downstream surfacing/joint detailing. 

Inputs → Outputs: curves, lengthFactor, tolerance → sub-curves. 

Typical use: Prep node geometry for SurfX or MultiPipe/SubD pipelines. 

Core API: Intersect.Intersection.CurveCurve, Curve.Split, Curve.Trim, Curve.LengthParameter.

MidX — Mid-span sub-curves
What it does: Splits curves at all intersections and creates centered, scaled sub-curves per segment for mid-span bracing/connectors. 

Inputs → Outputs: curves, lengthFactor → sub-curves. 

Core API: Intersect.Intersection.CurveCurve, Curve.Split, Curve.Trim, Curve.LengthParameter. 

SurfX — Surface patches from curve groups
What it does: Groups intersecting sub-curves (e.g., SubX output), orients/loops them, and lofts/join-rebuilds surfaces for 3D node skins. 

Inputs → Outputs: subCurves, tolerance, flipNext?, linearizeEdges?, rebuildU/V → Brep surfaces. 

Core API: Brep.CreateFromLoft, Brep.JoinBreps, NURBS rebuild, domain tweaks for stability. 

DiagonalX — Diagonal truss subdivision
What it does: Clusters endpoints into nodes, generates (a)symmetric “fractal” subdivision points, and connects them into X-pattern truss beams. 

Inputs → Outputs: subX segments, subdivisionCount, fractalFactor, asym?, betweenTrussCount → beam curves. 

Core steps: node clustering → perpendicular frames → primary & additional diagonals. 

TeethXL — Finger-joint edges for laser cut
What it does: Builds “teeth” along opposite edges of rectangles and unions them into a fabrication-ready boundary (laser-friendly). 

Inputs → Outputs: rect, divisions, spacing, subLength → composite boundary + tooth curves. 

Notes: Used in stool/bench joinery iterations; updates with geometry. 

TeethXC — CNC-ready teeth with arc reliefs
What it does: TeethXL variant that bridges teeth with arcs/fillets so inside corners are routable (no impossible sharp corners). 

Inputs → Outputs: same as TeethXL → composite boundary with arcs (top/bottom), per-edge tooth sets. 

UniXL — Bridge nodes between two longest edges
What it does: Auto-selects the two longest input curves, pairs subdivision points, generates connector “teeth” (trapezoids + holes), trims long edges, and outputs a fitness vector. 

Inputs → Outputs: curves, values1/values2 or numSubdivisions, flipCurve2?, rectWidth/rectHeight, topLength/topWidthFactor, holeDiameter → segments, nodes (polyline), circles, centers, fitness, trimmed long segments. 

UniXC — CNC variant of UniXL (filleted nodes)
What it does: Extends UniXL with tool-diameter-aware fillets and positive/negative node variants; keeps fitness/validation logic. 

Inputs → Outputs: UniXL params + arcRadius, reverseNodes? → cleaned segments, node polylines (±), circles, centers, fitness, trimmed edges.


### Known Issue — UniXC
**NodePart1 output doesn’t behave as expected when `reverseNodes = false`.** This is a standard malfunction we’re tracking for UniXC; please consider contributing a fix.



## License

- **Code (custom scripts in `/src`)**: Apache License 2.0. See [`LICENSE`](./LICENSE).
- See [`NOTICE`](./NOTICE) for project attribution and trademarks.
- Dependencies (e.g., RhinoCommon, Grasshopper) are **not** included and are subject to their own licenses and terms.

## Warranty, Liability, and Support

Provided **“AS IS”**, without warranty or support, and with no guarantee of fitness for any purpose. See the warranty/liability sections of the Apache 2.0 license. Use at your own risk.  
Read more on applying Apache 2.0 correctly: <https://www.apache.org/legal/apply-license.html>

## Usage

- Requires Rhino/Grasshopper installed.
- Import scripts from `/src` into a C# Script component or compile into a GH plugin per your workflow.

## Trademarks

Rhino, Rhinoceros, and Grasshopper are trademarks of Robert McNeel & Associates. They are not affiliated with nor do they endorse this project. 
