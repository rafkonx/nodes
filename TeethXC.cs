//TeethXC
// This script generates a polycurve representing the outer boundary of a rectangle with "teeth" or sub-rectangles extending outward from two opposite edges.
// The teeth are created along the edges of the rectangle, with specified divisions and spacing.
// The script is designed to work within the Grasshopper environment for Rhino. 
// Rafail Konstantinidis, Part of MSc Interaction Generative Design at the Hellenic Open University, Greece.
//provided as is, no warranty, no liability, no support, no guarantee of fitness for any purpose.


using System;
using System.Collections.Generic;
using Rhino;
using Rhino.Geometry;
using Grasshopper.Kernel;

// The Script_Instance class defines a custom script for digital fabrication.
// It creates "teeth" along selected edges of a rectangle and bridges them with arcs.
// This is useful for generating complex boundaries or joints in digital fabrication workflows.
public class Script_Instance : GH_ScriptInstance
{
  /// <summary>
  /// The main method executed when the script runs.
  /// It takes a rectangle, divides its top and bottom edges into segments,
  /// generates protruding teeth along these segments, and then builds a composite boundary.
  /// Outputs:
  ///   A: The final composite boundary curve.
  ///   B: The list of curves for the top teeth.
  ///   C: The list of curves for the bottom teeth.
  /// </summary>
  private void RunScript(
		Rectangle3d rect,
		int divisions,
		double spacing,
		double subLength,
		ref object A,
		ref object B,
		ref object C)
  {
    // Ensure that the number of divisions is at least one.
    divisions = Math.Max(divisions, 1);
    
    // Extract the four corners of the rectangle.
    Point3d ptA = rect.Corner(0); // Typically the top-left corner.
    Point3d ptB = rect.Corner(1); // Top-right corner.
    Point3d ptC = rect.Corner(2); // Bottom-right corner.
    Point3d ptD = rect.Corner(3); // Bottom-left corner.
    Point3d rectCenter = rect.Center; // The center of the rectangle.

    // Calculate the inward vectors for the top and bottom edges.
    // The inward vector is determined by the direction from the edge’s midpoint to the rectangle's center.
    Vector3d topInward = GetInwardVector(ptA, ptB, rectCenter);
    Vector3d botInward = GetInwardVector(ptC, ptD, rectCenter);

    // Create teeth along the top and bottom edges.
    // Each tooth is created by dividing the edge into segments and then generating three line segments:
    // left side, tip, and right side.
    List<LineCurve> teethTop = Create3EdgeTeeth(ptA, ptB, divisions, spacing, subLength, topInward);
    List<LineCurve> teethBot = Create3EdgeTeeth(ptC, ptD, divisions, spacing, subLength, botInward);

    // Build composite curves for the top and bottom edges.
    // This function combines the teeth segments with connecting arcs to form a continuous edge.
    PolyCurve compTop = BuildCompositeEdge(ptA, ptB, teethTop, topInward);
    PolyCurve compBot = BuildCompositeEdge(ptC, ptD, teethBot, botInward);

    // Construct the final boundary curve by combining:
    // - The composite top edge,
    // - A straight line from the top-right to the bottom-right corner,
    // - The composite bottom edge,
    // - A straight line from the bottom-left back to the top-left corner.
    PolyCurve finalBoundary = new PolyCurve();
    finalBoundary.Append(compTop);
    finalBoundary.Append(new LineCurve(ptB, ptC));
    finalBoundary.Append(compBot);
    finalBoundary.Append(new LineCurve(ptD, ptA));
    // Close the polycurve to form a continuous loop.
    finalBoundary.MakeClosed(RhinoMath.ZeroTolerance);

    // Output the final boundary and the individual teeth curves.
    A = finalBoundary; // The complete boundary with arcs and teeth.
    B = teethTop;      // The list of top edge teeth.
    C = teethBot;      // The list of bottom edge teeth.
  }

  /// <summary>
  /// Computes an inward pointing vector relative to a given edge.
  /// The vector is determined by subtracting the midpoint of the edge from the rectangle's center.
  /// A fallback method is provided in case the vector is too small to be unitized.
  /// </summary>
  private Vector3d GetInwardVector(Point3d start, Point3d end, Point3d center)
  {
    // Calculate the vector from the midpoint of the edge to the center.
    Vector3d inward = center - (start + end) / 2.0;
    // Try to normalize the vector.
    if (!inward.Unitize())
    {
      // If normalization fails (e.g., the vector is nearly zero),
      // use the cross product of the edge direction with the Z-axis as a fallback.
      Vector3d edgeDir = end - start;
      if (edgeDir.Unitize())
        inward = Vector3d.CrossProduct(edgeDir, Vector3d.ZAxis);
      else
        inward = Vector3d.YAxis; // Default to the Y-axis if all else fails.
    }
    return inward;
  }

  /// <summary>
  /// Creates teeth along a specified edge by dividing it into segments.
  /// Each tooth is made up of three line segments:
  /// - Left side: from the base start to the tooth tip start.
  /// - Tip: the top edge connecting the two tip points.
  /// - Right side: from the tooth tip end to the base end.
  /// </summary>
  private List<LineCurve> Create3EdgeTeeth(Point3d start, Point3d end, 
                                         int divisions, double spacing, 
                                         double subLength, Vector3d inward)
  {
    List<LineCurve> teeth = new List<LineCurve>();
    // Divide the edge into evenly spaced points.
    List<Point3d> pts = DivideEdge(start, end, divisions);

    // Iterate through each segment of the divided edge.
    for (int i = 0; i < divisions; i++)
    {
      // Define the start and end of the current segment.
      Point3d segStart = pts[i];
      Point3d segEnd = pts[i + 1];
      
      // Determine the direction of the segment.
      Vector3d edgeDir = segEnd - segStart;
      if (!edgeDir.Unitize()) continue; // Skip if the segment is invalid.

      // Adjust the segment by a spacing factor to create a base for the tooth.
      Point3d baseStart = segStart + edgeDir * (spacing / 2);
      Point3d baseEnd = segEnd - edgeDir * (spacing / 2);
      
      // Calculate the tip positions of the tooth by projecting the base points inward.
      Point3d tipStart = baseStart + inward * subLength;
      Point3d tipEnd = baseEnd + inward * subLength;
      
      // Create the three edges of the tooth:
      // Left edge: from baseStart to tipStart.
      teeth.Add(new LineCurve(baseStart, tipStart));
      // Tip edge: from tipStart to tipEnd.
      teeth.Add(new LineCurve(tipStart, tipEnd));
      // Right edge: from tipEnd to baseEnd.
      teeth.Add(new LineCurve(tipEnd, baseEnd));
    }
    return teeth;
  }

  /// <summary>
  /// Builds a composite edge curve by combining the teeth segments with connecting arcs.
  /// The resulting polycurve starts at the provided start point, includes all teeth with arcs between them,
  /// and ends at the provided end point.
  /// </summary>
  private PolyCurve BuildCompositeEdge(Point3d startPt, Point3d endPt, 
                                      List<LineCurve> teeth, Vector3d inward)
  {
    PolyCurve composite = new PolyCurve();
    // If there are no teeth, simply create a straight line between start and end.
    if (teeth.Count == 0)
    {
      composite.Append(new LineCurve(startPt, endPt));
      return composite;
    }

    // Connect the starting point to the beginning of the first tooth with an arc.
    composite.Append(CreateArc(startPt, teeth[0].PointAtStart, inward));

    // Iterate over the teeth in groups of three (each tooth is made of three segments).
    for (int i = 0; i < teeth.Count; i += 3)
    {
      // Append the three segments for the current tooth.
      composite.Append(teeth[i]);     // Left edge of the tooth.
      composite.Append(teeth[i + 1]); // Tip edge.
      composite.Append(teeth[i + 2]); // Right edge.
      
      // If another tooth exists, bridge the current tooth to the next one with an arc.
      if (i < teeth.Count - 3)
      {
        composite.Append(CreateArc(
          teeth[i + 2].PointAtEnd, // End of current tooth.
          teeth[i + 3].PointAtStart, // Start of the next tooth.
          inward
        ));
      }
    }

    // Connect the end of the last tooth to the overall end point with an arc.
    composite.Append(CreateArc(
      teeth[teeth.Count - 1].PointAtEnd,
      endPt,
      inward
    ));

    return composite;
  }

  /// <summary>
  /// Creates an arc curve between two points.
  /// The arc's curvature is defined by an "inward" vector which offsets the midpoint.
  /// If the two points are too close or if the arc is invalid, a straight line is returned instead.
  /// </summary>
  private Curve CreateArc(Point3d start, Point3d end, Vector3d inward)
  {
    // If the start and end points are nearly identical, return a straight line.
    if (start.DistanceTo(end) < RhinoMath.ZeroTolerance)
      return new LineCurve(start, end);

    // Calculate the chord and its midpoint between the two points.
    Vector3d chord = end - start;
    Point3d mid = start + chord / 2;
    // Define the arc's radius as half the chord length.
    double radius = chord.Length / 2;
    // Offset the midpoint by the inward vector scaled by the radius to form the arc's peak.
    Point3d arcMid = mid + inward * radius;
    
    // Create the arc using the start, offset midpoint, and end.
    Arc arc = new Arc(start, arcMid, end);
    // Return the arc as a NurbsCurve if valid; otherwise, fall back to a straight line.
    return arc.IsValid ? arc.ToNurbsCurve() : new LineCurve(start, end);
  }

  /// <summary>
  /// Divides an edge between two points into a specified number of equal segments.
  /// Returns a list of points that includes both the start and end points.
  /// </summary>
  private List<Point3d> DivideEdge(Point3d start, Point3d end, int divisions)
  {
    List<Point3d> pts = new List<Point3d>();
    // Calculate the step vector for each division.
    Vector3d step = (end - start) / divisions;
    // Generate and add each point along the edge.
    for (int i = 0; i <= divisions; i++)
      pts.Add(start + step * i);
    return pts;
  }
}
