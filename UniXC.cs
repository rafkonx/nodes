//UniXC
// This script generates fabrication nodes based on input curves and parameters.
// It subdivides the curves, creates connectors, and processes them into fabrication nodes.
// The nodes are then trimmed, filleted, and combined into segments for further processing.
// The script also evaluates the fitness of the generated nodes based on various criteria.
// The output includes segments, teeth, circles, centers, instructions, and node parts.
// The script is designed to work within the Rhino and Grasshopper environment, utilizing RhinoCommon and Grasshopper libraries.
// Rafail Konstantinidis, Part of MSc Interaction Generative Design at the Hellenic Open University, Greece.
// provided as is, no warranty, no liability, no support, no guarantee of fitness for any purpose.

//NodePart1 output dont behave as expected when reverseNodes is false 

using System;
using System.Collections.Generic;
using System.Linq;
using Rhino;
using Rhino.Geometry;
using Rhino.Geometry.Intersect;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;

public class Script_Instance : GH_ScriptInstance
{
  // Tolerance for geometric operations
  private double _tolerance;
  private const double _angleTolerance = RhinoMath.DefaultAngleTolerance;

  // Data container for each fabrication node
  private class FabricationNodeData
  {
    public Polyline ToothPolyline { get; set; }
    public List<Line> ConnectingSegments { get; set; } = new List<Line>();
    public bool IsPositive { get; set; }
    public Point3d Center { get; set; }
    public Circle Hole { get; set; }
    public Point3d BaseLeftPt { get; set; }
    public Point3d BaseRightPt { get; set; }
    public Point3d TopLeftPt { get; set; }
    public Point3d TopRightPt { get; set; }
    public Curve OriginalConnector { get; set; }
    public double ConnectorParam { get; set; }
  }

  // Comparer for deduplicating split parameters
  private class ToleranceComparer : IEqualityComparer<double>
  {
    private readonly double _tol;
    public ToleranceComparer(double tol) { _tol = Math.Abs(tol); }
    public bool Equals(double x, double y) => Math.Abs(x - y) < _tol;
    public int GetHashCode(double obj) => obj.GetHashCode();
  }

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
		double arcRadius,
		double arcTolerance,
		bool reverseNodes,
		ref object segments,
		ref object teeth,
		ref object Circles,
		ref object centres,
		ref object instruction,
		ref object nodepart1,
		ref object nodepart2,
		ref object LongSeg)
  {
    bool revNodes = reverseNodes;
    _tolerance = RhinoDoc.ActiveDoc?.ModelAbsoluteTolerance ?? 0.001;

    // --- Input validation ---
    if (inputCurves == null || inputCurves.Count < 2)
    {
      AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Need at least two curves.");
      return;
    }
    inputCurves = inputCurves.Where(c => c != null && c.IsValid).ToList();
    if (inputCurves.Count < 2)
    {
      AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Not enough valid curves after filtering.");
      return;
    }
    if (rectsPerLine < 1) rectsPerLine = 1;

    // --- Primary edges ---
    List<Curve> sorted = inputCurves
      .OrderByDescending(c => c.GetLength())
      .Select(c => c.DuplicateCurve())
      .ToList();
    Curve edge1 = sorted[0];
    Curve edge2 = sorted[1];
    List<Curve> forbiddenCurves = sorted.Skip(2).ToList();
    if (flipCurve2 && !edge2.Reverse())
      AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "Failed to reverse second edge.");

    // --- Subdivision points ---
    List<Point3d> pts1, pts2;
    bool custom = Values1 != null && Values2 != null
                  && Values1.Count == Values2.Count && Values1.Count > 0;
    if (custom)
    {
      if (Values1.Any(v => v < 0 || v > 1) || Values2.Any(v => v < 0 || v > 1))
      {
        AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Custom subdivision parameters must be in [0,1].");
        return;
      }
      Interval d1 = edge1.Domain, d2 = edge2.Domain;
      pts1 = Values1.Select(t => edge1.PointAt(d1.ParameterAt(t))).ToList();
      pts2 = Values2.Select(t => edge2.PointAt(d2.ParameterAt(t))).ToList();
    }
    else
    {
      pts1 = GetSubdivisionPoints(edge1, numSubdivisions);
      pts2 = GetSubdivisionPoints(edge2, numSubdivisions);
      if (pts1 == null || pts2 == null) return;
    }

    // --- Connectors ---
    List<Line> connectors = CreateConnectors(pts1, pts2);
    if (connectors == null) return;
    if (!custom && connectors.Count > 2)
      TrimEdgeConnectors(ref connectors);

    // --- Process connectors → nodes + raw segments ---
    List<FabricationNodeData> allNodeData = new List<FabricationNodeData>();
    List<Line> allRawSegments = new List<Line>();
    foreach (var ln in connectors)
    {
      var crv = ln.ToNurbsCurve();
      if (crv == null || !crv.IsValid) continue;
      ProcessConnector(crv, rectsPerLine, rectWidth, rectHeight, topLength, topWidthFactor, holeDiameter,
                       ref allNodeData, ref allRawSegments);
    }

    // --- Clean raw segments ---
    List<Curve> tempNodeCurves = allNodeData
      .Select(nd => CombineToothAndSegments(nd, _tolerance))
      .Where(c => c != null && c.IsValid)
      .ToList();
    segments = CleanRawSegments(
      allRawSegments,
      tempNodeCurves,
      (arcRadius > _tolerance) ? arcRadius * arcTolerance : _tolerance * 2);

    // --- Fitness score ---
    instruction = EvaluateFitness(
      connectors,
      allNodeData.Select(nd => nd.ToothPolyline).ToList(),
      forbiddenCurves,
      edge1, edge2,
      pts1, pts2);

    // --- Final node parts: trim + arc for positives only ---
    List<Curve> positive = new List<Curve>();
    List<Curve> negative = new List<Curve>();
    List<Curve> forEdgeTrim = new List<Curve>();

    foreach (var nd in allNodeData)
    {
      Curve combined = CombineToothAndSegments(nd, _tolerance);
      if (combined == null || !combined.IsValid) continue;
      forEdgeTrim.Add(combined.DuplicateCurve());

      if (arcRadius > _tolerance)
      {
        bool filletInner = (nd.IsPositive == revNodes);
        var fil = FilletCombinedNodeManual_PointBased(combined, nd, arcRadius, _tolerance, filletInner);
        if (fil != null && fil.IsValid)
          combined = fil;
      }

      // --- Trim to base ---
      combined = TrimNodeCurve(combined, nd.BaseLeftPt, nd.BaseRightPt, _tolerance);

      // --- Arc extension for positive only ---
      if (nd.IsPositive && !revNodes)
      {
        var ext = BuildFullSequenceForNode(nd, combined, arcRadius, _tolerance, arcTolerance);
        if (ext != null && ext.IsValid)
          combined = ext;
      }

      if (nd.IsPositive) positive.Add(combined);
      else               negative.Add(combined);
    }

    nodepart1 = positive;
    nodepart2 = negative;

    // --- Trim primary edges by nodes ---
    LongSeg = TrimPrimaryEdges(
      new List<Curve> { edge1, edge2 },
      forEdgeTrim,
      _tolerance);
  }

  // --- Helper methods ---

  private List<Point3d> GetSubdivisionPoints(Curve crv, int count)
  {
    if (crv == null || !crv.IsValid) return null;
    count = Math.Max(1, count);
    double[] t = crv.DivideByCount(count, true);
    if (t == null || t.Length == 0)
      return new List<Point3d> { crv.PointAtStart, crv.PointAtEnd };
    return t.Select(p => crv.PointAt(p)).ToList();
  }

  private List<Line> CreateConnectors(List<Point3d> a, List<Point3d> b)
  {
    if (a == null || b == null) return null;
    int n = Math.Min(a.Count, b.Count);
    var list = new List<Line>();
    for (int i = 0; i < n; i++)
      if (a[i] != b[i])
        list.Add(new Line(a[i], b[i]));
    return list;
  }

  private void TrimEdgeConnectors(ref List<Line> c)
  {
    if (c != null && c.Count > 2)
    {
      c.RemoveAt(c.Count - 1);
      c.RemoveAt(0);
    }
  }

  private void ProcessConnector(
    Curve crv,
    int divisions,
    double baseW, double baseH,
    double topExt, double topW,
    double holeD,
    ref List<FabricationNodeData> nodes,
    ref List<Line> rawSegs)
  {
    if (crv == null || !crv.IsValid || crv.GetLength() < _tolerance) return;
    divisions = Math.Max(1, divisions);
    double[] t = crv.DivideByCount(divisions, true);
    if (t == null || t.Length < 2) return;

    Vector3d dir = crv.TangentAtStart;
    if (!dir.Unitize()) dir = crv.PointAtEnd - crv.PointAtStart;
    dir.Unitize();
    Vector3d norm = ComputeFabricationNormal(dir);
    if (norm.IsTiny()) return;

    var occupied = new List<Interval>();
    var onThis = new List<FabricationNodeData>();

    for (int i = 1; i < t.Length - 1; i++)
    {
      bool pos = ((i - 1) % 2 == 0);
      var nd = CreateFabricationNode(
        crv, t[i], baseW, baseH, topExt, topW, holeD,
        dir, norm, pos, ref occupied);
      if (nd != null) onThis.Add(nd);
    }

    if (!onThis.Any()) return;

    var free = CalculateFreeParameterIntervals(occupied, crv.Domain);
    foreach (var iv in free)
    {
      Point3d p0 = crv.PointAt(iv.Min), p1 = crv.PointAt(iv.Max);
      if (p0 != p1) rawSegs.Add(new Line(p0, p1));
    }

    AssociateSegmentsWithNodes(rawSegs, onThis);
    nodes.AddRange(onThis);
  }

  private Vector3d ComputeFabricationNormal(Vector3d dir)
  {
    if (dir.IsTiny(_tolerance)) return Vector3d.Zero;
    Vector3d n = Vector3d.CrossProduct(dir, Vector3d.ZAxis);
    if (n.Unitize()) return n;
    n = Vector3d.CrossProduct(dir, Vector3d.XAxis);
    if (n.Unitize()) return n;
    n = Vector3d.CrossProduct(dir, Vector3d.YAxis);
    n.Unitize();
    return n;
  }

  private FabricationNodeData CreateFabricationNode(
    Curve crv, double t,
    double bw, double bh,
    double te, double tw,
    double hd,
    Vector3d dir, Vector3d norm,
    bool pos,
    ref List<Interval> occupied)
  {
    Point3d basePt = crv.PointAt(t);
    double sign = pos ? 1.0 : -1.0;
    Vector3d off = norm * (bh / 2.0 * sign);
    Point3d center = basePt + off;
    Plane pl = new Plane(center, dir, norm * sign);
    if (!pl.IsValid) return null;

    Point3d bl, br, tl, tr;
    var poly = CreateLaserNodePolyline(pl, bw, bh, te, tw, out bl, out br, out tl, out tr);
    if (poly == null) return null;

    var hole = new Circle(pl, hd / 2.0);

    // mark occupied interval
    double paramPerLen = 1.0;
    var ders = crv.DerivativeAt(t, 1);
    if (ders != null && ders.Length > 1 && ders[1].Length > _tolerance)
      paramPerLen = 1.0 / ders[1].Length;
    else
    {
      double len = crv.GetLength(), dom = crv.Domain.Length;
      if (len > _tolerance && Math.Abs(dom) > _tolerance) paramPerLen = dom / len;
      else if (len > _tolerance) paramPerLen = 1.0 / len;
    }
    double halfW = (bw / 2.0) * paramPerLen;
    occupied.Add(new Interval(t - halfW, t + halfW));

    return new FabricationNodeData
    {
      ToothPolyline    = poly,
      IsPositive       = pos,
      Center           = center,
      Hole             = hole,
      BaseLeftPt       = bl,
      BaseRightPt      = br,
      TopLeftPt        = tl,
      TopRightPt       = tr,
      OriginalConnector= crv,
      ConnectorParam   = t
    };
  }

  private Polyline CreateLaserNodePolyline(
    Plane pl,
    double bw, double bh,
    double te, double wr,
    out Point3d bl, out Point3d br,
    out Point3d tl, out Point3d tr)
  {
    bl = br = tl = tr = Point3d.Unset;
    wr = RhinoMath.Clamp(wr, 0.01, 10.0);
    double halfB = bw / 2.0, halfT = halfB * wr;

    br = pl.PointAt( halfB, -bh/2.0 );
    tr = pl.PointAt( halfT,  bh/2.0 + te );
    tl = pl.PointAt(-halfT,  bh/2.0 + te );
    bl = pl.PointAt(-halfB, -bh/2.0 );

    var pts = new List<Point3d> { bl, tl, tr, br };
    var poly = new Polyline(pts);
    if (!poly.IsValid || poly.Length < _tolerance || poly.Count != 4) return null;
    return poly;
  }

  private List<Interval> CalculateFreeParameterIntervals(List<Interval> occupied, Interval dom)
  {
    var free = new List<Interval>();
    if (occupied == null || !occupied.Any())
    {
      if (dom.IsValid && dom.Length > _tolerance) free.Add(dom);
      return free;
    }
    var merged = MergeIntervals(occupied, _tolerance);
    double cur = dom.Min;
    foreach (var iv in merged)
    {
      double s = Math.Max(dom.Min, iv.Min), e = Math.Min(dom.Max, iv.Max);
      if (s > cur + _tolerance) free.Add(new Interval(cur, s));
      cur = Math.Max(cur, e);
    }
    if (cur < dom.Max - _tolerance) free.Add(new Interval(cur, dom.Max));
    return free;
  }

  private List<Interval> MergeIntervals(List<Interval> ints, double tol)
  {
    if (ints == null || ints.Count <= 1) return ints ?? new List<Interval>();
    var valid = ints.Where(i => i.IsValid).OrderBy(i => i.Min).ToList();
    var merged = new List<Interval> { valid[0] };
    foreach (var next in valid.Skip(1))
    {
      var last = merged.Last();
      if (next.Min <= last.Max + tol)
        merged[merged.Count - 1] = new Interval(last.Min, Math.Max(last.Max, next.Max));
      else
        merged.Add(next);
    }
    return merged;
  }

  private void AssociateSegmentsWithNodes(List<Line> segs, List<FabricationNodeData> nodes)
  {
    if (segs == null || nodes == null) return;
    double tolSq = _tolerance * _tolerance;
    foreach (var nd in nodes)
    {
      foreach (var s in segs)
      {
        if (!s.IsValid) continue;
        bool nearBL = s.From.DistanceToSquared(nd.BaseLeftPt) < tolSq || s.To.DistanceToSquared(nd.BaseLeftPt) < tolSq;
        bool nearBR = s.From.DistanceToSquared(nd.BaseRightPt) < tolSq || s.To.DistanceToSquared(nd.BaseRightPt) < tolSq;
        if ((nearBL || nearBR) && !nd.ConnectingSegments.Contains(s))
          nd.ConnectingSegments.Add(s);
      }
    }
  }

  private Curve CombineToothAndSegments(FabricationNodeData nd, double tol)
  {
    var tooth = nd.ToothPolyline;
    if (tooth == null || !tooth.IsValid || tooth.Count != 4) return null;
    double tolSq = tol * tol;
    var pBL = nd.BaseLeftPt;
    var pBR = nd.BaseRightPt;
    Line segL = Line.Unset, segR = Line.Unset;
    foreach (var s in nd.ConnectingSegments)
    {
      if (!s.IsValid) continue;
      bool fBL = s.From.DistanceToSquared(pBL) < tolSq, tBL = s.To.DistanceToSquared(pBL) < tolSq;
      bool fBR = s.From.DistanceToSquared(pBR) < tolSq, tBR = s.To.DistanceToSquared(pBR) < tolSq;
      if ((fBL || tBL) && !segL.IsValid) segL = s;
      if ((fBR || tBR) && !segR.IsValid) segR = s;
      if (segL.IsValid && segR.IsValid) break;
    }
    var pts = new List<Point3d>();
    if (segL.IsValid) pts.Add(segL.From.DistanceToSquared(pBL) < tolSq ? segL.To : segL.From);
    pts.Add(pBL);
    pts.Add(nd.TopLeftPt);
    pts.Add(nd.TopRightPt);
    pts.Add(pBR);
    if (segR.IsValid) pts.Add(segR.From.DistanceToSquared(pBR) < tolSq ? segR.To : segR.From);
    if (pts.Count < 2) return null;
    var pl = new Polyline(pts);
    if (!pl.IsValid || pl.Length < tol) return null;
    return new PolylineCurve(pl);
  }

  private Curve FilletCombinedNodeManual_PointBased(
    Curve cc, FabricationNodeData nd, double r, double tol, bool filletInner)
  {
    if (cc == null || !cc.IsValid || r <= tol) return cc;
    if (!cc.TryGetPolyline(out Polyline pl) || pl.Count < 3) return cc;

    Point3d pt1 = filletInner ? nd.BaseLeftPt : nd.TopLeftPt;
    Point3d pt2 = filletInner ? nd.BaseRightPt : nd.TopRightPt;
    var idxs = new List<int>();
    int i1 = pl.ClosestIndex(pt1);
    if (i1 > 0 && i1 < pl.Count - 1 && pl[i1].DistanceToSquared(pt1) < tol * tol) idxs.Add(i1);
    int i2 = pl.ClosestIndex(pt2);
    if (i2 > 0 && i2 < pl.Count - 1 && i2 != i1 && pl[i2].DistanceToSquared(pt2) < tol * tol) idxs.Add(i2);
    if (!idxs.Any()) return cc;
    idxs.Sort();

    var newPts = new List<Point3d> { pl[0] };
    int ptr = 0;
    for (int i = 1; i < pl.Count - 1; i++)
    {
      if (ptr < idxs.Count && i == idxs[ptr])
      {
        var corner = pl[i];
        var prev   = pl[i - 1];
        var next   = pl[i + 1];
        var v1 = prev - corner; var v2 = next - corner;
        double l1 = v1.Length, l2 = v2.Length;
        if (!v1.Unitize() || !v2.Unitize() || l1 < tol || l2 < tol) { newPts.Add(corner); ptr++; continue; }
        double ang = Vector3d.VectorAngle(v1, v2);
        if (ang < _angleTolerance || ang > Math.PI - _angleTolerance) { newPts.Add(corner); ptr++; continue; }
        double tanHalf = Math.Tan(ang / 2.0);
        if (Math.Abs(tanHalf) < tol) { newPts.Add(corner); ptr++; continue; }
        double off = r / tanHalf; off = Math.Min(off, l1 / 2.0); off = Math.Min(off, l2 / 2.0);
        if (off <= tol) { newPts.Add(corner); ptr++; continue; }
        var ptA = corner + v1 * off;
        var ptB = corner + v2 * off;
        var arc = new Arc(ptA, corner, ptB);
        if (arc.IsValid)
        {
          newPts.Add(ptA);
          var arcCrv = arc.ToNurbsCurve();
          int samples = Math.Min(20, Math.Max(3, (int)Math.Ceiling(arc.Length / (r * 0.5)) + 1));
          var ps = arcCrv.DivideByCount(samples, false);
          foreach (var t in ps)
          {
            var ap = arcCrv.PointAt(t);
            if (newPts.Last().DistanceTo(ap) > tol / 2) newPts.Add(ap);
          }
          if (newPts.Last().DistanceTo(ptB) > tol / 2) newPts.Add(ptB);
        }
        else newPts.Add(corner);
        ptr++;
      }
      else newPts.Add(pl[i]);
    }
    newPts.Add(pl.Last());
    if (newPts.Count < 2) return cc;
    var finalPl = new Polyline(newPts);
    if (!finalPl.IsValid || finalPl.Length < tol) return cc;
    return new PolylineCurve(finalPl);
  }

  /// <summary>
  /// Builds an arc extension from trimmed node‐end to the closest point on the connector.
  /// </summary>
  private Curve BuildFullSequenceForNode(
    FabricationNodeData nd,
    Curve nodeCrv,
    double connectingArcRadius,
    double tolerance,
    double arcTolerance)
  {
    if (nodeCrv == null || !nodeCrv.IsValid ||
        nd.OriginalConnector == null || !nd.OriginalConnector.IsValid)
      return nodeCrv;

    // Find connector parameters nearest to node‐curve endpoints
    Point3d nStart = nodeCrv.PointAtStart;
    Point3d nEnd   = nodeCrv.PointAtEnd;

    nd.OriginalConnector.ClosestPoint(nStart, out double tStart);
    Point3d startPt = nd.OriginalConnector.PointAt(tStart);

    nd.OriginalConnector.ClosestPoint(nEnd, out double tEnd);
    Point3d endPt   = nd.OriginalConnector.PointAt(tEnd);

    double gapStart = startPt.DistanceTo(nStart);
    double gapEnd   = endPt  .DistanceTo(nEnd);
    double threshold = Math.Max(tolerance * 10.0, connectingArcRadius * arcTolerance);

    if (gapStart >= gapEnd && gapStart > threshold)
    {
      var arc = CreateConnectingArc(startPt, nStart, nd.Center, connectingArcRadius, false, tolerance);
      if (arc != null && arc.IsValid)
      {
        var joined = Curve.JoinCurves(new[] { arc, nodeCrv }, tolerance * 1.5, true);
        if (joined != null && joined.Length > 0) return joined[0];
      }
    }
    else if (gapEnd > threshold)
    {
      var arc = CreateConnectingArc(nEnd, endPt, nd.Center, connectingArcRadius, false, tolerance);
      if (arc != null && arc.IsValid)
      {
        var joined = Curve.JoinCurves(new[] { nodeCrv, arc }, tolerance * 1.5, true);
        if (joined != null && joined.Length > 0) return joined[0];
      }
    }

    return nodeCrv;
  }

  /// <summary>
  /// Creates a pure arc bulging toward refPt, or null if invalid.
  /// </summary>
  private Curve CreateConnectingArc(
    Point3d start,
    Point3d end,
    Point3d refPt,
    double radius,
    bool createOutwardArc,
    double tolerance)
  {
    if (start.DistanceTo(end) < tolerance) return null;

    Point3d mid = (start + end) * 0.5;
    Vector3d dir = refPt - mid;
    if (!dir.Unitize()) dir = Vector3d.ZAxis;
    if (createOutwardArc) dir.Reverse();
    Point3d bulgePt = mid + dir * radius;

    Arc arc = new Arc(start, bulgePt, end);
    return arc.IsValid ? arc.ToNurbsCurve() : null;
  }

  private Curve TrimNodeCurve(Curve nodeCrv, Point3d bl, Point3d br, double tol)
  {
    if (nodeCrv == null || !nodeCrv.IsValid) return nodeCrv;
    var baseLine = new LineCurve(new Line(bl, br));
    var inters = Intersection.CurveCurve(nodeCrv, baseLine, tol, tol);
    if (inters != null && inters.Count >= 2)
    {
      var ps = inters.Where(i => i.IsPoint).Select(i => i.ParameterA).OrderBy(x => x).ToArray();
      var splits = nodeCrv.Split(ps);
      if (splits != null)
      {
        if (splits.Length == 3) return splits[1];
        if (splits.Length == 2)
        {
          double d0 = DistanceToLineAverage(splits[0], baseLine);
          double d1 = DistanceToLineAverage(splits[1], baseLine);
          return d0 < d1 ? splits[0] : splits[1];
        }
      }
    }
    return nodeCrv;
  }

  private double DistanceToLineAverage(Curve crv, LineCurve line)
  {
    line.ClosestPoint(crv.PointAtStart, out double t0);
    Point3d p0 = line.PointAt(t0);
    line.ClosestPoint(crv.PointAtEnd, out double t1);
    Point3d p1 = line.PointAt(t1);
    return (crv.PointAtStart.DistanceTo(p0) + crv.PointAtEnd.DistanceTo(p1)) * 0.5;
  }

  private List<Curve> TrimPrimaryEdges(List<Curve> edges, List<Curve> nodes, double tol)
  {
    var result = new List<Curve>();
    var comp = new ToleranceComparer(tol);

    foreach (var edge in edges)
    {
      if (edge == null || !edge.IsValid) continue;
      var parms = new List<double> { edge.Domain.Min, edge.Domain.Max };
      foreach (var nc in nodes)
      {
        if (nc == null || !nc.IsValid) continue;
        var inters = Intersection.CurveCurve(edge, nc, tol, tol);
        if (inters != null)
          parms.AddRange(inters.Where(i => i.IsPoint).Select(i => i.ParameterA));
      }
      var distinct = parms.Distinct(comp).OrderBy(x => x).ToArray();
      if (distinct.Length <= 2)
      {
        result.Add(edge.DuplicateCurve());
      }
      else
      {
        try
        {
          var parts = edge.Split(distinct);
          if (parts != null && parts.Length > 0)
            result.AddRange(parts.Where(s => s != null && s.IsValid && s.GetLength() > tol));
          else
            result.Add(edge.DuplicateCurve());
        }
        catch
        {
          result.Add(edge.DuplicateCurve());
        }
      }
    }

    return result;
  }

  private double EvaluateFitness(
    List<Line> connectors,
    List<Polyline> nodes,
    List<Curve> forbidden,
    Curve e1, Curve e2,
    List<Point3d> p1, List<Point3d> p2)
  {
    double tol = _tolerance, pen = 0.0;
    double minA = 88, maxA = 93;
    double fA = 0.1, fI = 10.0, fD = 0.2, fP = 0.2;
    double minD = 5.0, pTol = 5.0;

    int n = Math.Min(connectors.Count, Math.Min(p1.Count, p2.Count));
    for (int i = 0; i < n; i++)
    {
      var c = connectors[i]; if (!c.IsValid) continue;
      var dir = c.Direction; dir.Unitize();

      if (!e1.ClosestPoint(p1[i], out double t1, Math.Max(tol * 100, e1.GetLength() * 0.1))) continue;
      var tan1 = e1.TangentAt(t1); tan1.Unitize();

      if (!e2.ClosestPoint(p2[i], out double t2, Math.Max(tol * 100, e2.GetLength() * 0.1))) continue;
      var tan2 = e2.TangentAt(t2); tan2.Unitize();

      double a1 = RhinoMath.ToDegrees(Vector3d.VectorAngle(tan1, dir));
      double a2 = RhinoMath.ToDegrees(Vector3d.VectorAngle(tan2, dir));
      if (a1 > 90) a1 = 180 - a1;
      if (a2 > 90) a2 = 180 - a2;

      if (a1 < minA) pen += (minA - a1) * fA;
      else if (a1 > maxA) pen += (a1 - maxA) * fA;
      if (a2 < minA) pen += (minA - a2) * fA;
      else if (a2 > maxA) pen += (a2 - maxA) * fA;
    }

    var geo = new List<Curve>();
    geo.AddRange(connectors.Select(l => l.ToNurbsCurve()).Where(c => c != null && c.IsValid));
    geo.AddRange(nodes.Select(p => p.ToNurbsCurve()).Where(c => c != null && c.IsValid));

    foreach (var g in geo)
      foreach (var f in forbidden)
        if (f != null && f.IsValid)
        {
          var inters = Intersection.CurveCurve(g, f, tol, tol);
          if (inters != null && inters.Count > 0) pen += inters.Count * fI;
        }

    int samples = 10;
    foreach (var c in connectors)
    {
      var cc = c.ToNurbsCurve(); if (cc == null || !cc.IsValid) continue;
      var ps = cc.DivideByCount(samples - 1, true); if (ps == null) continue;
      foreach (var f in forbidden)
      {
        if (f == null || !f.IsValid) continue;
        double minSq = double.MaxValue, bestC = 0, bestF = 0;
        foreach (var tc in ps)
        {
          var pt = cc.PointAt(tc);
          if (f.ClosestPoint(pt, out double tf))
          {
            var fp = f.PointAt(tf);
            double d2 = pt.DistanceToSquared(fp);
            if (d2 < minSq) { minSq = d2; bestC = tc; bestF = tf; }
          }
        }
        double md = Math.Sqrt(minSq);
        if (md < minD) pen += (minD - md) * fD;
        var fTan = f.TangentAt(bestF); fTan.Unitize();
        var cTan = cc.TangentAt(bestC); cTan.Unitize();
        double ang = RhinoMath.ToDegrees(Vector3d.VectorAngle(fTan, cTan));
        if (ang > 90) ang = 180 - ang;
        double dev = Math.Abs(ang - 90);
        if (dev > pTol) pen += (dev - pTol) * fP;
      }
    }

    return Math.Max(0.0, pen);
  }

  private List<Line> CleanRawSegments(
    List<Line> rawSegs,
    List<Curve> nodeCurves,
    double threshold)
  {
    var outList = new List<Line>();
    foreach (var s in rawSegs)
    {
      if (!s.IsValid) continue;
      var mid = s.PointAt(0.5);
      bool discard = false;
      foreach (var nc in nodeCurves)
      {
        if (nc == null || !nc.IsValid) continue;
        if (nc.ClosestPoint(mid, out double t))
        {
          if (mid.DistanceTo(nc.PointAt(t)) < threshold)
          {
            discard = true;
            break;
          }
        }
      }
      if (!discard) outList.Add(s);
    }
    return outList;
  }
}
