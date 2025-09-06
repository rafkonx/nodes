// MidX
// This script processes a list of curves to generate segments by splitting them at intersections.
// Create curves at the middle point of the segments, scaled by a factor.
// It scales the segments based on a specified length factor and outputs the resulting segments.
// The script ensures that segments are valid and have a minimum length before adding them to the output.
// Rafail Konstantinidis, Part of MSc Interaction Generative Design at the Hellenic Open University, Greece.
   
using System;
using System.Collections.Generic;
using Rhino;
using Rhino.Geometry;
using Rhino.Geometry.Intersect;

public class Script_Instance : GH_ScriptInstance
{
    private void RunScript(
		List<Curve> InputCurves,
		double LengthFactor,
		ref object OutputSegments)
    {
        // Initialize output
        List<Curve> resultSegments = new List<Curve>();
        double tolerance = RhinoDoc.ActiveDoc.ModelAbsoluteTolerance;
        
        // Process each curve
        foreach (Curve crv in InputCurves)
        {
            if (crv == null || !crv.IsValid) continue;

            List<double> splitParams = new List<double>();

            // Find intersections with all other curves
            foreach (Curve otherCrv in InputCurves)
            {
                if (crv == otherCrv) continue;
                
                var intersections = Intersection.CurveCurve(crv, otherCrv, tolerance, tolerance);
                foreach (var intersection in intersections)
                {
                    if (intersection.IsPoint)
                    {
                        splitParams.Add(intersection.ParameterA);
                    }
                }
            }

            // Add curve endpoints and sort
            splitParams.Add(crv.Domain.Min);
            splitParams.Add(crv.Domain.Max);
            splitParams.Sort();

            // Split and process segments
            Curve[] segments = crv.Split(splitParams);
            foreach (Curve segment in segments)
            {
                if (segment == null || segment.GetLength() < tolerance) continue;

                // Create scaled segment
                Curve scaled = ScaleSegmentFromCenter(segment, LengthFactor);
                if (scaled != null && scaled.IsValid)
                {
                    resultSegments.Add(scaled);
                }
            }
        }

        OutputSegments = resultSegments;
    }

    private Curve ScaleSegmentFromCenter(Curve segment, double factor)
    {
        // Scale segment length while keeping center position
        double originalLength = segment.GetLength();
        double newLength = originalLength * factor;
        
        if (newLength < RhinoMath.ZeroTolerance)
            return null;

        double trimAmount = (originalLength - newLength) / 2;
        
        if (segment.LengthParameter(trimAmount, out double tStart) &&
            segment.LengthParameter(originalLength - trimAmount, out double tEnd))
        {
            return segment.Trim(tStart, tEnd);
        }
        
        return segment.DuplicateCurve();
    }
}