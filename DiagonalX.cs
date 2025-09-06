// DiagonalX
// This script processes a list of input curves, generating fractal points and truss beams based on specified parameters.// It clusters the curves by their endpoints, generates fractal points along each curve,
// and creates beams connecting these points. The script also supports asymmetrical subdivision and additional truss beam subdivisions.// The output consists of a list of fractal points and a collection of truss beams.
// The script is designed to run in the context of a Grasshopper C# component in Rhino
// Rafail Konstantinidis, Part of MSc Interaction Generative Design at the Hellenic Open University, Greece.


using System;
using System.Collections.Generic;
using Rhino;
using Rhino.Geometry;
using System.Linq;

public class Script_Instance : GH_ScriptInstance
{
    private void RunScript(
		List<Curve> InputSegments,
		int SubdivisionCount,
		double FractalFactor,
		bool AsymSubdivision,
		int BetweenTrussSubdivisionCount,
		ref object FractalPoints,
		ref object TrussBeams)
    {
        double tol = RhinoDoc.ActiveDoc.ModelAbsoluteTolerance;

        // Collect all curve endpoints
        List<Point3d> allEndpoints = InputSegments
            .Where(c => c != null && c.IsValid)
            .SelectMany(c => new[] { c.PointAtStart, c.PointAtEnd })
            .ToList();

        // Cluster endpoints into unique nodes using tolerance
        List<Point3d> nodes = new List<Point3d>();
        foreach (Point3d pt in allEndpoints)
        {
            if (!nodes.Any(n => n.DistanceTo(pt) < tol))
                nodes.Add(pt);
        }

        // Group curves by their connected nodes
        Dictionary<Point3d, List<Curve>> groups = new Dictionary<Point3d, List<Curve>>(new Point3dComparer(tol));
        foreach (Point3d node in nodes)
        {
            List<Curve> connectedCurves = InputSegments
                .Where(c => c != null && c.IsValid &&
                    (c.PointAtStart.DistanceTo(node) < tol || c.PointAtEnd.DistanceTo(node) < tol))
                .ToList();
            if (connectedCurves.Count > 0)
                groups[node] = connectedCurves;
        }

        // Process groups and generate fractal points/beams
        List<List<List<Point3d>>> groupFractalPoints = new List<List<List<Point3d>>>();
        List<Curve> beams = new List<Curve>();

        foreach (var group in groups)
        {
            Point3d node = group.Key;
            List<Curve> curves = group.Value;

            // Orient curves so that the start is at the node
            foreach (Curve crv in curves)
            {
                if (crv.PointAtEnd.DistanceTo(node) < tol)
                    crv.Reverse();
            }

            // Generate fractal points for each curve in the group
            List<List<Point3d>> curvePoints = curves
                .Select(c => GenerateFractalPoints(c, SubdivisionCount, FractalFactor, AsymSubdivision, tol))
                .ToList();
            groupFractalPoints.Add(curvePoints);

            // Create main beams connecting corresponding fractal points
            if (curvePoints.Count > 1)
            {
                int minPoints = curvePoints.Min(c => c.Count);
                for (int i = 0; i < minPoints; i++)
                {
                    for (int j = 0; j < curvePoints.Count; j++)
                    {
                        for (int k = j + 1; k < curvePoints.Count; k++)
                        {
                            Point3d ptA = curvePoints[j][i];
                            Point3d ptB = curvePoints[k][i];
                            if (ptA.DistanceTo(ptB) > tol)
                                beams.Add(new LineCurve(ptA, ptB));
                        }
                    }
                }

                // Additional subdivision of the truss beams:
                // For each adjacent pair of fractal nodes on the curves,
                // subdivide the segment and add extra diagonal beams on both sides (X-shape)
                if (BetweenTrussSubdivisionCount > 0 && minPoints >= 2)
                {
                    for (int i = 0; i < minPoints - 1; i++)
                    {
                        for (int j = 0; j < curvePoints.Count; j++)
                        {
                            for (int k = j + 1; k < curvePoints.Count; k++)
                            {
                                Point3d A = curvePoints[j][i];
                                Point3d B = curvePoints[j][i + 1];
                                Point3d C = curvePoints[k][i];
                                Point3d D = curvePoints[k][i + 1];

                                // Subdivide each segment into (BetweenTrussSubdivisionCount + 2) points (including endpoints)
                                List<Point3d> subSeg1 = SubdivideLine(A, B, BetweenTrussSubdivisionCount + 2);
                                List<Point3d> subSeg2 = SubdivideLine(C, D, BetweenTrussSubdivisionCount + 2);

                                for (int s = 1; s < subSeg1.Count - 1; s++)
                                {
                                    // Connect corresponding subdivision points
                                    Point3d P = subSeg1[s];
                                    Point3d Q = subSeg2[s];
                                    if (P.DistanceTo(Q) > tol)
                                        beams.Add(new LineCurve(P, Q));

                                    // Create X-diagonals: connect subSeg1[s] to subSeg2[s+1]...
                                    if (s + 1 < subSeg1.Count)
                                    {
                                        Line diag1 = new Line(subSeg1[s], subSeg2[s + 1]);
                                        if (diag1.Length > tol)
                                            beams.Add(new LineCurve(diag1));
                                    }
                                    // ...and connect subSeg1[s+1] to subSeg2[s]
                                    if (s + 1 < subSeg2.Count)
                                    {
                                        Line diag2 = new Line(subSeg1[s + 1], subSeg2[s]);
                                        if (diag2.Length > tol)
                                            beams.Add(new LineCurve(diag2));
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

        // Assign outputs
        FractalPoints = groupFractalPoints;
        TrussBeams = beams;
    }

    private List<Point3d> GenerateFractalPoints(Curve crv, int subdivCount, double fractalFactor, bool asym, double tol)
    {
        List<Point3d> pts = new List<Point3d>();
        double tStart = crv.Domain.Min;
        double tEnd = crv.Domain.Max;

        pts.Add(crv.PointAt(tStart)); // Start at the node

        if (!asym)
        {
            // Even subdivision across the entire curve
            for (int i = 1; i <= subdivCount; i++)
            {
                double t = tStart + (tEnd - tStart) * (i / (double)subdivCount);
                ProcessPoint(crv, t, fractalFactor, tol, pts);
            }
        }
        else
        {
            // Asymmetrical subdivision starting from the midpoint
            double tMid = tStart + 0.5 * (tEnd - tStart);
            pts.Add(crv.PointAt(tMid));
            for (int i = 1; i <= subdivCount; i++)
            {
                double t = tMid + (tEnd - tMid) * (i / (double)subdivCount);
                ProcessPoint(crv, t, fractalFactor, tol, pts);
            }
        }

        pts.Add(crv.PointAt(tEnd)); // Always add the endpoint
        return pts;
    }

    private void ProcessPoint(Curve crv, double t, double fractalFactor, double tol, List<Point3d> pts)
    {
        Point3d pt = crv.PointAt(t);
        if (crv.PerpendicularFrameAt(t, out Plane frame))
        {
            // Use the YAxis as a perpendicular direction
            Vector3d perp = frame.YAxis;
            double offset = fractalFactor * crv.GetLength() * 0.1;
            pt += perp * offset * (pts.Count % 2 == 0 ? 1 : -1);
        }
        pts.Add(pt);
    }

    private List<Point3d> SubdivideLine(Point3d A, Point3d B, int count)
    {
        List<Point3d> pts = new List<Point3d>();
        for (int i = 0; i < count; i++)
        {
            double t = i / (double)(count - 1);
            pts.Add(A + (B - A) * t);
        }
        return pts;
    }

    private class Point3dComparer : IEqualityComparer<Point3d>
    {
        private readonly double _tolerance;
        public Point3dComparer(double tolerance) => _tolerance = tolerance;

        public bool Equals(Point3d p1, Point3d p2) => p1.DistanceTo(p2) <= _tolerance;
        
        public int GetHashCode(Point3d p)
        {
            return Math.Round(p.X / _tolerance).GetHashCode() ^
                   Math.Round(p.Y / _tolerance).GetHashCode();
        }
    }
}
