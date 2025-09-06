//UniXL 
// This script generates node curves representing the inner and connecting boundary of along the two bigger curves from set of curves.  
//"teeth" or sub-rectangles are possitioned by setting Values (Gene pool component is convienient) that extending throughout its length by parametarising the longest curves.
// The script is designed to work within the Grasshopper environment for Rhino.
// Rafail Konstantinidis, Part of MSc Interaction Generative Design at the Hellenic Open University, Greece.
//provided as is, no warranty, no liability, no support, no guarantee of fitness for any purpose.

using System;
using System.Collections.Generic;
using System.Linq;
using Rhino;
using Rhino.Geometry;
using Rhino.Geometry.Intersect;
using Grasshopper.Kernel;

public class Script_Instance : GH_ScriptInstance
{
  /// <summary>
  /// Main entry point.
  /// 
  /// Inputs:
  ///   inputCurves: at least two curves. The two longest become the primary edges.
  ///   Values1: (optional) normalized subdivision parameters (0–1) for the first edge.
  ///   Values2: (optional) normalized subdivision parameters for the second edge (must match count with Values1).
  ///   numSubdivisions: if no custom values are provided, evenly subdivide using this count.
  ///   flipCurve2: if true, the second edge is reversed before subdivision.
  ///   rectsPerLine, rectWidth, rectHeight, topLength, topWidthFactor, holeDiameter:
  ///       parameters for generating the node (trapezoidal tooth) geometry.
  /// 
  /// Outputs:
  ///   segments: the trimmed connector lines.
  ///   teeth: the laser node (tooth) polylines.
  ///   Circles: the connection holes as curves.
  ///   centres: the centres of the laser nodes.
  ///   instruction: the fitness value (penalty); lower is better.
  ///   nodes: the union of segments and teeth.
  ///   LongSeg: the trimmed portions of the two primary edges (longest curves) obtained by splitting them at intersections with nodes.
  /// 
  /// (The original evaluation criteria remain unchanged.)
  /// </summary>
  private void RunScript(
		List<Curve> inputCurves,
		List<double> Values1,
		List<double> Values2,
		int numSubdivisions,
		bool flipCurve2,
		int rectsPerLine,
		double rectWidth,
		double rectHeight,
		double topLength,
		double topWidthFactor,
		double holeDiameter,
		ref object segments,
		ref object teeth,
		ref object Circles,
		ref object centres,
		ref object instruction,
		ref object nodes,
		ref object LongSeg)
  {
    // 1. Validate inputs.
    if (inputCurves == null || inputCurves.Count < 2)
    {
      AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "At least two curves are required.");
      return;
    }
    
    // 2. Determine primary edges and forbidden curves.
    var sortedCurves = inputCurves.OrderByDescending(c => c.GetLength()).ToList();
    Curve edge1 = sortedCurves[0];
    Curve edge2 = sortedCurves[1];
    // All remaining curves become forbidden.
    List<Curve> forbiddenCurves = sortedCurves.Skip(2).ToList();
    
    // If requested, flip the second edge.
    if (flipCurve2)
      edge2.Reverse();
    
    // 3. Compute subdivision points for each edge.
    List<Point3d> pts1, pts2;
    bool useCustom = (Values1 != null && Values2 != null &&
                      Values1.Count > 0 && Values1.Count == Values2.Count);
    
    if (useCustom)
    {
      Interval dom1 = edge1.Domain;
      Interval dom2 = edge2.Domain;
      pts1 = Values1.Select(p => edge1.PointAt(dom1.ParameterAt(p))).ToList();
      pts2 = Values2.Select(p => edge2.PointAt(dom2.ParameterAt(p))).ToList();
    }
    else
    {
      pts1 = GetSubdivisionPoints(edge1, numSubdivisions);
      pts2 = GetSubdivisionPoints(edge2, numSubdivisions);
    }
    
    // Create connectors by pairing corresponding points.
    List<Line> connectors = CreateConnectors(pts1, pts2);
    // For even subdivisions, trim the first and last connectors.
    if (!useCustom)
      TrimEdgeConnectors(ref connectors);
    
    // 4. Process connectors to generate node (tooth) geometry.
    List<Polyline> laserNodes = new List<Polyline>();
    List<Circle> connectionHoles = new List<Circle>();
    List<Point3d> nodeCenters = new List<Point3d>();
    List<Line> trimmedConnectors = new List<Line>();
    
    try
    {
      foreach (Line connector in connectors)
      {
        ProcessConnector(
          connector,
          rectsPerLine,
          rectWidth,
          rectHeight,
          topLength,
          topWidthFactor,
          holeDiameter,
          ref laserNodes,
          ref connectionHoles,
          ref nodeCenters,
          ref trimmedConnectors);
      }
    }
    catch (Exception ex)
    {
      AddRuntimeMessage(GH_RuntimeMessageLevel.Error, ex.Message);
      return;
    }
    
    // 5. Set standard outputs.
    segments = trimmedConnectors;
    teeth = laserNodes;
    Circles = connectionHoles.Select(c => c.ToNurbsCurve()).ToList<Curve>();
    centres = nodeCenters;
    
    double fitness = EvaluateFitness(connectors, laserNodes, forbiddenCurves, edge1, edge2, pts1, pts2);
    instruction = fitness;
    
    // --- New Step 1: Join teeth with segments to form nodes.
    List<Curve> nodesList = new List<Curve>();
    // Convert trimmed connector lines (segments) to curves.
    foreach(Line seg in trimmedConnectors)
      nodesList.Add(seg.ToNurbsCurve());
    // Convert each laser node (polyline) to a NurbsCurve.
    foreach(Polyline poly in laserNodes)
      nodesList.Add(poly.ToNurbsCurve());
    nodes = nodesList;
    
    // --- New Step 2: Trim the primary edges (edge1 and edge2) using intersections with the nodes.
    List<Curve> longSegList = new List<Curve>();
    // Use the union of nodesList as the trimming geometry.
    foreach (Curve primEdge in new Curve[] { edge1, edge2 })
    {
      // Accumulate intersection parameters.
      List<double> tParams = new List<double>();
      foreach (Curve nodeCurve in nodesList)
      {
        CurveIntersections inters = Intersection.CurveCurve(primEdge, nodeCurve, Rhino.RhinoDoc.ActiveDoc.ModelAbsoluteTolerance, Rhino.RhinoDoc.ActiveDoc.ModelAbsoluteTolerance);
        if (inters != null && inters.Count > 0)
        {
          foreach (IntersectionEvent ie in inters)
          {
            tParams.Add(ie.ParameterA);
          }
        }
      }
      tParams.Sort();
      // If we have at least one intersection, split the curve.
      if (tParams.Count > 0)
      {
        Curve[] splits = primEdge.Split(tParams.ToArray());
        if (splits != null)
        {
          // Optionally, you could choose the longest split segment or join segments.
          // Here we simply add all split segments.
          longSegList.AddRange(splits);
        }
        else
        {
          longSegList.Add(primEdge.DuplicateCurve());
        }
      }
      else
      {
        // If no intersections, output the full curve.
        longSegList.Add(primEdge.DuplicateCurve());
      }
    }
    LongSeg = longSegList;
  }
  
  // --- HELPER METHODS ---
  
  // Evenly subdivides a curve into a given count.
  private List<Point3d> GetSubdivisionPoints(Curve crv, int count)
  {
    double[] tParams = crv.DivideByCount(count, true);
    return tParams.Select(t => crv.PointAt(t)).ToList();
  }
  
  // Creates connectors by pairing corresponding points.
  private List<Line> CreateConnectors(List<Point3d> pts1, List<Point3d> pts2)
  {
    int minCount = Math.Min(pts1.Count, pts2.Count);
    return Enumerable.Range(0, minCount)
                     .Select(i => new Line(pts1[i], pts2[i]))
                     .ToList();
  }
  
  // Trims the first and last connectors (used when subdivisions are generated evenly).
  private void TrimEdgeConnectors(ref List<Line> connectors)
  {
    if (connectors.Count > 2)
    {
      connectors.RemoveAt(0);
      connectors.RemoveAt(connectors.Count - 1);
    }
  }
  
  // Processes a connector by dividing it into nodes and generating the node (tooth) geometry.
  private void ProcessConnector(
    Line connector,
    int nodesPerLine,
    double baseWidth,
    double baseHeight,
    double topExtension,
    double topWidthRatio,
    double holeSize,
    ref List<Polyline> nodes,
    ref List<Circle> holes,
    ref List<Point3d> centers,
    ref List<Line> trimmedLines)
  {
    Curve crv = connector.ToNurbsCurve();
    double[] tParams = crv.DivideByCount(nodesPerLine, true);
    
    Vector3d lineDir = connector.Direction;
    lineDir.Unitize();
    
    // Compute a normal for offset.
    Vector3d normal = ComputeFabricationNormal(lineDir);
    List<Interval> occupiedSections = new List<Interval>();
    
    for (int i = 1; i < tParams.Length - 1; i++)
    {
      CreateFabricationNode(
        crv,
        tParams[i],
        baseWidth,
        baseHeight,
        topExtension,
        topWidthRatio,
        holeSize,
        lineDir,
        normal,
        i,
        ref nodes,
        ref holes,
        ref centers,
        ref occupiedSections);
    }
    
    trimmedLines.AddRange(CalculateValidConnectors(crv, occupiedSections));
  }
  
  // Computes a normal vector based on a given direction.
  private Vector3d ComputeFabricationNormal(Vector3d direction)
  {
    Vector3d normal = Vector3d.CrossProduct(direction, Vector3d.ZAxis);
    if (!normal.Unitize())
    {
      normal = Vector3d.CrossProduct(direction, Vector3d.XAxis);
      normal.Unitize();
    }
    return normal;
  }
  
  // Creates a single laser node (tooth) at the given parameter along the connector.
  private void CreateFabricationNode(
    Curve connector,
    double tParam,
    double baseWidth,
    double baseHeight,
    double topExtension,
    double topWidthRatio,
    double holeSize,
    Vector3d lineDir,
    Vector3d normal,
    int index,
    ref List<Polyline> nodes,
    ref List<Circle> holes,
    ref List<Point3d> centers,
    ref List<Interval> occupiedSections)
  {
    Point3d basePoint = connector.PointAt(tParam);
    double offsetSign = (index % 2 == 0) ? 1 : -1;
    Vector3d heightOffset = normal * (baseHeight / 2 * offsetSign);
    
    Point3d center = basePoint + heightOffset;
    centers.Add(center);
    
    Plane nodePlane = new Plane(center, lineDir, offsetSign * normal);
    Polyline node = CreateLaserNode(nodePlane, baseWidth, baseHeight, topExtension, topWidthRatio);
    nodes.Add(node);
    
    Circle hole = new Circle(nodePlane, holeSize / 2);
    holes.Add(hole);
    
    double paramRange = connector.Domain.Max - connector.Domain.Min;
    double paramPerWidth = (baseWidth / 2) / connector.GetLength() * paramRange;
    occupiedSections.Add(new Interval(tParam - paramPerWidth, tParam + paramPerWidth));
  }
  
  // Constructs a polyline representing the laser node (tooth) geometry.
  private Polyline CreateLaserNode(Plane plane, double baseWidth, double baseHeight, double topExtension, double widthRatio)
  {
    widthRatio = Rhino.RhinoMath.Clamp(widthRatio, 0.1, 2.0);
    double halfBase = baseWidth / 2;
    double halfTop = halfBase * widthRatio;
    double baseY = -baseHeight / 2;
    
    return new Polyline(new[] {
      plane.PointAt(halfBase, baseY),
      plane.PointAt(halfTop, topExtension),
      plane.PointAt(-halfTop, topExtension),
      plane.PointAt(-halfBase, baseY)
    });
  }
  
  // Returns connector segments that are unoccupied.
  private List<Line> CalculateValidConnectors(Curve connector, List<Interval> occupied)
  {
    List<Interval> merged = MergeIntervals(occupied);
    List<Interval> freeSections = CalculateFreeSections(merged, connector.GetLength());
    return freeSections.Select(interval => 
      new Line(connector.PointAt(interval.Min), connector.PointAt(interval.Max))
    ).ToList();
  }
  
  // Merges overlapping intervals.
  private List<Interval> MergeIntervals(List<Interval> intervals)
  {
    if (!intervals.Any()) return intervals;
    intervals.Sort((a, b) => a.Min.CompareTo(b.Min));
    List<Interval> merged = new List<Interval> { intervals[0] };
    for (int i = 1; i < intervals.Count; i++)
    {
      Interval last = merged.Last();
      if (intervals[i].Min <= last.Max)
        merged[merged.Count - 1] = new Interval(last.Min, Math.Max(last.Max, intervals[i].Max));
      else
        merged.Add(intervals[i]);
    }
    return merged;
  }
  
  // Computes free intervals along a curve.
  private List<Interval> CalculateFreeSections(List<Interval> occupied, double totalLength)
  {
    List<Interval> free = new List<Interval>();
    double currentStart = 0;
    foreach (Interval interval in occupied.OrderBy(i => i.Min))
    {
      if (interval.Min > currentStart)
        free.Add(new Interval(currentStart, interval.Min));
      currentStart = interval.Max;
    }
    if (currentStart < totalLength)
      free.Add(new Interval(currentStart, totalLength));
    return free;
  }
  
  /// <summary>
  /// Evaluates fitness using three criteria:
  ///   (a) Angle penalty – For each connector, the angle between its direction and the tangent
  ///       of each primary edge (at the corresponding subdivision point) must be between 88° and 93°.
  ///       Deviations add a penalty scaled by angleFactor.
  ///   (b) Intersection penalty – Each intersection between any connector or any laser node (tooth)
  ///       and any forbidden curve adds a penalty scaled by intersectionFactor.
  ///   (c) Proximity & Perpendicularity penalty – For each connector and each forbidden curve,
  ///       the connector is sampled (10 points). If the minimum distance from these samples to the forbidden curve is less than minDist,
  ///       a penalty proportional to (minDist - distance) is added (scaled by distanceFactor). Also, at the closest sample,
  ///       the angle between the connector’s direction and the forbidden curve’s tangent should be 90°.
  ///       Any deviation beyond perpTol adds a penalty scaled by perpFactor.
  /// </summary>
  private double EvaluateFitness(
    List<Line> connectors,
    List<Polyline> nodes,
    List<Curve> forbiddenCurves,
    Curve edge1,
    Curve edge2,
    List<Point3d> pts1,
    List<Point3d> pts2)
  {
    double penalty = 0.0;
    double tol = Rhino.RhinoDoc.ActiveDoc != null ? Rhino.RhinoDoc.ActiveDoc.ModelAbsoluteTolerance : 0.001;
    
    // Scaling factors.
    double angleFactor = 0.1;
    double intersectionFactor = 0.1;
    double distanceFactor = 0.2; // For proximity penalty.
    double perpFactor = 0.2;     // For perpendicularity penalty.
    
    // Distance threshold and perpendicular tolerance.
    double minDist = 5.0;   // Minimum allowed distance from connector to forbidden curves.
    double perpTol = 5.0;   // Allowed deviation (in degrees) from perfect perpendicular (90°).
    
    // (a) Angle penalty for primary edges.
    for (int i = 0; i < connectors.Count; i++)
    {
      Line conn = connectors[i];
      
      double t1;
      edge1.ClosestPoint(pts1[i], out t1);
      Vector3d tan1 = edge1.TangentAt(t1);
      tan1.Unitize();
      
      double t2;
      edge2.ClosestPoint(pts2[i], out t2);
      Vector3d tan2 = edge2.TangentAt(t2);
      tan2.Unitize();
      
      Vector3d connDir = conn.Direction;
      connDir.Unitize();
      
      double angle1 = Rhino.RhinoMath.ToDegrees(Vector3d.VectorAngle(tan1, connDir));
      double angle2 = Rhino.RhinoMath.ToDegrees(Vector3d.VectorAngle(tan2, connDir));
      
      // Acceptable range: 88° to 93°.
      if (angle1 < 88)
        penalty += (88 - angle1) * angleFactor;
      else if (angle1 > 93)
        penalty += (angle1 - 93) * angleFactor;
      
      if (angle2 < 88)
        penalty += (88 - angle2) * angleFactor;
      else if (angle2 > 93)
        penalty += (angle2 - 93) * angleFactor;
    }
    
    // (b) Intersection penalty for connectors.
    foreach (Line conn in connectors)
    {
      Curve connCurve = conn.ToNurbsCurve();
      foreach (Curve f in forbiddenCurves)
      {
        CurveIntersections inters = Intersection.CurveCurve(connCurve, f, tol, tol);
        if (inters != null && inters.Count > 0)
          penalty += inters.Count * intersectionFactor;
      }
    }
    
    // (c) Intersection penalty for laser node (tooth) polylines.
    foreach (Polyline node in nodes)
    {
      Curve nodeCurve = node.ToNurbsCurve();
      foreach (Curve f in forbiddenCurves)
      {
        CurveIntersections inters = Intersection.CurveCurve(nodeCurve, f, tol, tol);
        if (inters != null && inters.Count > 0)
          penalty += inters.Count * intersectionFactor;
      }
    }
    
    // (d) Proximity & Perpendicularity penalty for each connector relative to each forbidden curve.
    // For each connector, sample 10 points uniformly.
    int numSamples = 10;
    foreach (Line conn in connectors)
    {
      Curve connCurve = conn.ToNurbsCurve();
      double connDomainLength = connCurve.Domain.Length;
      // For each forbidden curve:
      foreach (Curve f in forbiddenCurves)
      {
        double minSampleDist = double.MaxValue;
        double bestSampleParam = connCurve.Domain.Min;
        // Sample along the connector.
        for (int j = 0; j < numSamples; j++)
        {
          double t = connCurve.Domain.Min + (connDomainLength * j) / (numSamples - 1);
          Point3d samplePt = connCurve.PointAt(t);
          double tF;
          f.ClosestPoint(samplePt, out tF);
          Point3d closestPt = f.PointAt(tF);
          double d = samplePt.DistanceTo(closestPt);
          if (d < minSampleDist)
          {
            minSampleDist = d;
            bestSampleParam = t;
          }
        }
        // If the minimum distance is less than the threshold, add a penalty.
        if (minSampleDist < minDist)
          penalty += (minDist - minSampleDist) * distanceFactor;
        
        // At the sample point that gave minSampleDist, check perpendicularity.
        Point3d bestPt = connCurve.PointAt(bestSampleParam);
        double tF_best;
        f.ClosestPoint(bestPt, out tF_best);
        Vector3d fTan = f.TangentAt(tF_best);
        fTan.Unitize();
        Vector3d connDir = conn.Direction;
        connDir.Unitize();
        double angle = Rhino.RhinoMath.ToDegrees(Vector3d.VectorAngle(fTan, connDir));
        double deviation = Math.Abs(angle - 90);
        if (deviation > perpTol)
          penalty += (deviation - perpTol) * perpFactor;
      }
    }
    
    return penalty;
  }
}
