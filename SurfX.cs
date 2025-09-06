// SurfX
// This script processes a list of curves to generate surfaces by lofting them.
// It groups curves that share intersections, orders them circularly if needed,
// and creates lofted surfaces between pairs of curves. It also allows for optional
// flipping of curves and optimizes the resulting surfaces by reparameterizing them.
// The script is designed to work within the Grasshopper environment for Rhino.
// Rafail Konstantinidis, Part of MSc Interaction Generative Design at the Hellenic Open University, Greece.

using System;
using System.Collections.Generic;
using Rhino;
using Rhino.Geometry;
using Rhino.Geometry.Intersect;

public class Script_Instance : GH_ScriptInstance
{
  // New inputs: rebuildU and rebuildV for rebuilding factors; flipNext to optionally flip next curve.
  private void RunScript(
		List<Curve> inputCurves,
		double tolerance,
		bool linearizeEdges,
		bool flipNext,
		int rebuildU,
		int rebuildV,
		ref object A)
  {
    List<Brep> generatedSurfaces = new List<Brep>();

    if (inputCurves == null || inputCurves.Count < 2)
    {
      A = generatedSurfaces;
      return;
    }

    // Group curves that share intersections
    List<List<Curve>> groupedCurves = GroupCurvesByIntersection(inputCurves, tolerance);

    foreach (List<Curve> group in groupedCurves)
    {
      // For groups with 3 or more curves, order them circularly to form a closed loop.
      List<Curve> orderedCurves = OrderCurvesByIntersections(group, tolerance);
      int curveCount = orderedCurves.Count;
      
      // Loop through each pair using wrap-around (last pairs with first)
      for (int i = 0; i < curveCount; i++)
      {
        Curve curve1 = orderedCurves[i];
        Curve curve2 = orderedCurves[(i + 1) % curveCount];

        // Loft only if the curves actually intersect.
        var intersections = Intersection.CurveCurve(curve1, curve2, tolerance, tolerance);
        if (intersections == null || intersections.Count == 0)
          continue;

        Point3d interPt = intersections[0].PointA;
        curve1 = OrientCurveRelativeToIntersection(curve1, interPt, true, tolerance);
        curve2 = OrientCurveRelativeToIntersection(curve2, interPt, false, tolerance);
        
        // Optionally flip the next curve.
        curve2 = ApplyFlip(curve2, flipNext);

        // Create loft surface between the two curves.
        Brep[] loftSurface = Brep.CreateFromLoft(
          new List<Curve> { curve1, curve2 },
          Point3d.Unset,
          Point3d.Unset,
          LoftType.Normal,
          false
        );
        Brep finalSurface = GetJoinedSurface(loftSurface, tolerance);

        // Fallback to a straight loft if needed.
        if (finalSurface == null || !finalSurface.IsValid)
        {
          loftSurface = Brep.CreateFromLoft(
            new List<Curve> { curve1, curve2 },
            Point3d.Unset,
            Point3d.Unset,
            LoftType.Straight,
            false
          );
          finalSurface = GetJoinedSurface(loftSurface, tolerance);
        }

        // Optimize the surface quality by reparameterizing it using rebuildU and rebuildV.
        if (finalSurface != null && finalSurface.IsValid)
        {
          finalSurface = OptimizeSurfaceQuality(finalSurface, rebuildU, rebuildV, tolerance);
          generatedSurfaces.Add(finalSurface);
        }
      }
    }

    A = generatedSurfaces;
  }

  /// <summary>
  /// Optionally reverses the curve if flip is true.
  /// </summary>
  private Curve ApplyFlip(Curve curve, bool flip)
  {
    if (flip)
      curve.Reverse();
    return curve;
  }

  /// <summary>
  /// Joins lofted Breps (if more than one) and returns the first valid joined surface.
  /// </summary>
  private Brep GetJoinedSurface(Brep[] loftBreps, double tolerance)
  {
    if (loftBreps == null || loftBreps.Length == 0)
      return null;
    if (loftBreps.Length == 1)
      return loftBreps[0];

    Brep[] joined = Brep.JoinBreps(loftBreps, tolerance);
    if (joined != null && joined.Length > 0)
      return joined[0];
    return null;
  }

  /// <summary>
  /// Reparameterizes the first face of the Brep for improved surface quality.
  /// Forces the U and V domains to a slightly shrunken interval to avoid singular trimming domains.
  /// </summary>
  private Brep OptimizeSurfaceQuality(Brep brep, int rebuildU, int rebuildV, double tol)
  {
    if (brep == null || !brep.IsValid || brep.Faces.Count == 0)
      return brep;
    
    Surface srf = brep.Faces[0].DuplicateSurface();
    NurbsSurface nurbsSrf = srf.ToNurbsSurface();
    if (nurbsSrf == null || !nurbsSrf.IsValid)
      return brep;
    
    // Rebuild the surface with degree 3 in both directions using the provided factors.
    NurbsSurface rebuilt = nurbsSrf.Rebuild(3, 3, rebuildU, rebuildV);
    if (rebuilt == null || !rebuilt.IsValid)
      return brep;
    
    // Use a small epsilon (at least as big as tolerance) to shrink the domain.
    double eps = Math.Max(tol, 1e-3);
    rebuilt.SetDomain(0, new Interval(eps, 1 - eps));
    rebuilt.SetDomain(1, new Interval(eps, 1 - eps));
    
    // Validate the new domains.
    Interval newUDom = rebuilt.Domain(0);
    Interval newVDom = rebuilt.Domain(1);
    if (Math.Abs(newUDom.Length) < tol || Math.Abs(newVDom.Length) < tol)
      return brep; // Still degenerate, so return the original.
    
    Brep newBrep = Brep.CreateFromSurface(rebuilt);
    if (newBrep != null && newBrep.IsValid)
      return newBrep;
    
    return brep;
  }

  /// <summary>
  /// Reverses the curve if necessary so that the intersection point is at the desired end.
  /// </summary>
  private Curve OrientCurveRelativeToIntersection(Curve curve, Point3d intersectionPoint, bool intersectionAtStart, double tol)
  {
    double dStart = curve.PointAtStart.DistanceTo(intersectionPoint);
    double dEnd = curve.PointAtEnd.DistanceTo(intersectionPoint);

    if ((intersectionAtStart && dStart > dEnd + tol) || (!intersectionAtStart && dEnd > dStart + tol))
      curve.Reverse();

    return curve;
  }

  /// <summary>
  /// Groups curves that share intersections within a tolerance.
  /// </summary>
  private List<List<Curve>> GroupCurvesByIntersection(List<Curve> curves, double tolerance)
  {
    List<List<Curve>> groups = new List<List<Curve>>();
    HashSet<Curve> visited = new HashSet<Curve>();

    foreach (Curve curve in curves)
    {
      if (visited.Contains(curve))
        continue;

      Queue<Curve> queue = new Queue<Curve>();
      List<Curve> group = new List<Curve>();

      queue.Enqueue(curve);
      visited.Add(curve);

      while (queue.Count > 0)
      {
        Curve current = queue.Dequeue();
        group.Add(current);

        foreach (Curve candidate in curves)
        {
          if (!visited.Contains(candidate) &&
              Intersection.CurveCurve(current, candidate, tolerance, tolerance).Count > 0)
          {
            queue.Enqueue(candidate);
            visited.Add(candidate);
          }
        }
      }
      groups.Add(group);
    }
    return groups;
  }

  /// <summary>
  /// Orders curves based on intersections. For groups with three or more curves, a circular order is used.
  /// </summary>
  private List<Curve> OrderCurvesByIntersections(List<Curve> curves, double tolerance)
  {
    if (curves.Count >= 3)
      return OrderCurvesCircular(curves);
    else
      return new List<Curve>(curves);
  }

  /// <summary>
  /// Orders curves in a circular fashion based on the angles of their midpoints relative to their centroid.
  /// </summary>
  private List<Curve> OrderCurvesCircular(List<Curve> curves)
  {
    List<Curve> sortedCurves = new List<Curve>(curves);
    List<Point3d> midpoints = new List<Point3d>();

    foreach (Curve curve in sortedCurves)
      midpoints.Add(curve.PointAtNormalizedLength(0.5));

    Point3d centroid = new Point3d(0, 0, 0);
    foreach (Point3d pt in midpoints)
      centroid += pt;
    centroid /= midpoints.Count;

    sortedCurves.Sort((a, b) =>
    {
      Point3d ma = a.PointAtNormalizedLength(0.5);
      Point3d mb = b.PointAtNormalizedLength(0.5);
      double angleA = Math.Atan2(ma.Y - centroid.Y, ma.X - centroid.X);
      double angleB = Math.Atan2(mb.Y - centroid.Y, mb.X - centroid.X);
      return angleA.CompareTo(angleB);
    });
    return sortedCurves;
  }
}
