//TeethXL 
// This script generates a polycurve representing the outer boundary of a rectangle with "teeth" or sub-rectangles extending outward from two opposite edges.
// The teeth are created along the edges of the rectangle, with specified divisions and spacing.
// The script is designed to work within the Grasshopper environment for Rhino.
// Rafail Konstantinidis, Part of MSc Interaction Generative Design at the Hellenic Open University, Greece.
//provided as is, no warranty, no liability, no support, no guarantee of fitness for any purpose.



#region // begin main code
using System;
using System.Collections.Generic;
using Rhino;
using Rhino.Geometry;
using Grasshopper.Kernel; // For GH_RuntimeMessageLevel

public class Script_Instance : GH_ScriptInstance
{
  /// <summary>
  /// Main entry point.
  /// </summary>
  /// <param name="rect">
  /// Input rectangle (Rhino.Geometry.Rectangle3d). Corners assumed in order: A, B, C, D.
  /// </param>
  /// <param name="divisions">
  /// Number of segments to divide edges A–B and C–D.
  /// </param>
  /// <param name="spacing">
  /// Spacing value to inset each division segment.
  /// </param>
  /// <param name="subLength">
  /// Length that each sub-rectangle (tooth) extends outward from the main rectangle.
  /// </param>
  /// <param name="A">
  /// Output: A polycurve representing the outer boundary (union) of the trimmed main rectangle with the sub-rectangles.
  /// </param>
  private void RunScript(
		Rectangle3d rect,
		int divisions,
		double spacing,
		double subLength,
		ref object A)
  {
    // Ensure divisions is at least 1.
    divisions = Math.Max(divisions, 1);
    
    // Get corners of the main rectangle.
    Point3d ptA = rect.Corner(0);
    Point3d ptB = rect.Corner(1);
    Point3d ptC = rect.Corner(2);
    Point3d ptD = rect.Corner(3);
    Point3d center = rect.Center;
    
    // --- Create sub-rectangles ("tooths") along edge A-B ---
    List<Rectangle3d> subRectangles_AB = new List<Rectangle3d>();
    List<Point3d> ptsAB = DivideEdge(ptA, ptB, divisions);
    
    // Outward direction for edge A-B: reverse vector from edge midpoint to center.
    Point3d midAB = new Point3d((ptA.X + ptB.X)/2.0, (ptA.Y + ptB.Y)/2.0, (ptA.Z + ptB.Z)/2.0);
    Vector3d insideDirAB = center - midAB;
    insideDirAB.Unitize();
    Vector3d outwardAB = -insideDirAB;
    
    for (int i = 0; i < divisions; i++)
    {
      Point3d p0 = ptsAB[i];
      Point3d p1 = ptsAB[i + 1];
      Vector3d baseDir = p1 - p0;
      double segLength = baseDir.Length;
      if(segLength <= RhinoMath.ZeroTolerance) continue;
      baseDir.Unitize();
      
      // Inset each end by spacing/2.
      Point3d newP0 = p0 + baseDir * (spacing / 2.0);
      Point3d newP1 = p1 - baseDir * (spacing / 2.0);
      double adjustedLength = newP0.DistanceTo(newP1);
      
      // Define a plane for the tooth: origin at newP0, x-axis along adjusted base, y-axis = outward.
      Plane planeAB = new Plane(newP0, baseDir, outwardAB);
      
      // Build the tooth as a Rectangle3d.
      Rectangle3d toothAB = new Rectangle3d(planeAB, new Interval(0, adjustedLength), new Interval(0, subLength));
      subRectangles_AB.Add(toothAB);
    }
    // Remove first and last tooth from the A-B set.
    if (subRectangles_AB.Count > 2)
    {
      subRectangles_AB.RemoveAt(0);
      subRectangles_AB.RemoveAt(subRectangles_AB.Count - 1);
    }
    else
      subRectangles_AB.Clear();
    
    // --- Create sub-rectangles ("tooths") along edge C-D ---
    List<Rectangle3d> subRectangles_CD = new List<Rectangle3d>();
    List<Point3d> ptsCD = DivideEdge(ptC, ptD, divisions);
    ptsCD.Reverse(); // Reverse so segments correspond.
    Point3d midCD = new Point3d((ptC.X + ptD.X)/2.0, (ptC.Y + ptD.Y)/2.0, (ptC.Z + ptD.Z)/2.0);
    Vector3d insideDirCD = center - midCD;
    insideDirCD.Unitize();
    Vector3d outwardCD = -insideDirCD;
    
    for (int i = 0; i < divisions; i++)
    {
      Point3d p0 = ptsCD[i];
      Point3d p1 = ptsCD[i + 1];
      Vector3d baseDir = p1 - p0;
      double segLength = baseDir.Length;
      if(segLength <= RhinoMath.ZeroTolerance) continue;
      baseDir.Unitize();
      
      Point3d newP0 = p0 + baseDir * (spacing / 2.0);
      Point3d newP1 = p1 - baseDir * (spacing / 2.0);
      double adjustedLength = newP0.DistanceTo(newP1);
      
      Plane planeCD = new Plane(newP0, baseDir, outwardCD);
      Rectangle3d toothCD = new Rectangle3d(planeCD, new Interval(0, adjustedLength), new Interval(0, subLength));
      subRectangles_CD.Add(toothCD);
    }
    if (subRectangles_CD.Count > 2)
    {
      subRectangles_CD.RemoveAt(0);
      subRectangles_CD.RemoveAt(subRectangles_CD.Count - 1);
    }
    else
      subRectangles_CD.Clear();
    
    // --- Convert main rectangle and sub-rectangles into closed curves ---
    // Create a closed polyline for the main rectangle.
    Polyline mainPolyline = new Polyline(new List<Point3d> { ptA, ptB, ptC, ptD, ptA });
    Curve mainCurve = mainPolyline.ToNurbsCurve();
    
    // Convert each tooth into a closed curve.
    List<Curve> toothCurves = new List<Curve>();
    foreach (Rectangle3d r in subRectangles_AB)
    {
      Curve c = r.ToNurbsCurve();
      toothCurves.Add(c);
    }
    foreach (Rectangle3d r in subRectangles_CD)
    {
      Curve c = r.ToNurbsCurve();
      toothCurves.Add(c);
    }
    
    // Prepare a list of curves for Boolean union.
    List<Curve> allCurves = new List<Curve>();
    allCurves.Add(mainCurve);
    allCurves.AddRange(toothCurves);
    
    double tol = Rhino.RhinoDoc.ActiveDoc != null ? Rhino.RhinoDoc.ActiveDoc.ModelAbsoluteTolerance : 0.001;
    Curve[] unionCurves = Curve.CreateBooleanUnion(allCurves, tol);
    
    if (unionCurves == null || unionCurves.Length == 0)
    {
      AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "Curve Boolean union failed.");
      A = null;
      return;
    }
    
    // If multiple curves were returned, join them.
    Curve joined = Curve.JoinCurves(unionCurves, tol).Length > 0 ?
                   Curve.JoinCurves(unionCurves, tol)[0] : unionCurves[0];
    
    // Output the resulting curve (should be the outer boundary).
    A = joined;
  }
  
  /// <summary>
  /// Helper function to divide a line segment into (divisions+1) equally spaced points.
  /// </summary>
  private List<Point3d> DivideEdge(Point3d start, Point3d end, int divisions)
  {
    List<Point3d> pts = new List<Point3d>();
    for (int i = 0; i <= divisions; i++)
    {
      double t = (double)i / divisions;
      pts.Add(Lerp(start, end, t));
    }
    return pts;
  }
  
  /// <summary>
  /// Custom linear interpolation for Point3d.
  /// </summary>
  private Point3d Lerp(Point3d a, Point3d b, double t)
  {
    double x = a.X + t * (b.X - a.X);
    double y = a.Y + t * (b.Y - a.Y);
    double z = a.Z + t * (b.Z - a.Z);
    return new Point3d(x, y, z);
  }
}
#endregion
