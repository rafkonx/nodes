// SubX is a Grasshopper component that processes a list of curves
// to find intersection nodes and create subcurves extending from those nodes.
// It generates two subcurves for each intersection node: one extending backward and one extending forward
// based on a specified length factor. The component is designed to work with polyhedral edges or similar shapes
// where curves meet at nodes, allowing for flexible manipulation of the resulting segments.// The script is written in C# and is intended to be used within the Grasshopper environment for
// Rhino 3D modeling software. It utilizes RhinoCommon for geometry operations and intersection calculations.
// Rafail Konstantinidis, Part of MSc Interaction Generative Design at the Hellenic Open University, Greece.


using System;
using System.Collections.Generic;
using Rhino;
using Rhino.Geometry;
using Rhino.Geometry.Intersect;

public class Script_Instance : GH_ScriptInstance
{
  /// <summary>
  /// For each input curve, the script finds the intersection (connection) nodes.
  /// Then, for each intersection node it creates two subcurves:
  /// one extending backward (left) from the node and one extending forward (right).
  /// The subcurve length is a fraction (LengthFactor) of the available segment length.
  /// </summary>
  /// <param name="InputCurves">
  /// A list of curves. These may be edges of polyhedra or similar shapes that meet at nodes.
  /// </param>
  /// <param name="LengthFactor">
  /// A factor (between 0 and 1) that determines how far along the available segment the subcurve will extend.
  /// For example, if LengthFactor is 0.5 then each subcurve will have half the length available from the node.
  /// </param>
  /// <param name="OutputSegments">
  /// The list of subcurves (connection nodes) generated from each intersection.
  /// </param>
  private void RunScript(
		List<Curve> InputCurves,
		double LengthFactor,
		ref object OutputSegments)
  {
    // This list will hold all the subcurves we generate.
    List<Curve> resultSegments = new List<Curve>();
    double tolerance = RhinoDoc.ActiveDoc.ModelAbsoluteTolerance;
    double minLength = tolerance * 10; // segments shorter than this are ignored

    // Validate input.
    if (InputCurves == null || InputCurves.Count == 0 || LengthFactor <= 0)
    {
      OutputSegments = resultSegments;
      return;
    }

    // Process each curve individually.
    foreach (Curve crv in InputCurves)
    {
      if (crv == null || !crv.IsValid)
      {
        Print("Skipped invalid curve");
        continue;
      }

      // 1. Find raw intersection parameters along this curve.
      //    (These come directly from CurveCurve intersections with every other curve.)
      List<double> rawIntersections = new List<double>();
      foreach (Curve other in InputCurves)
      {
        if (other == null || other == crv)
          continue;

        var events = Intersection.CurveCurve(crv, other, tolerance, tolerance);
        if (events != null)
        {
          foreach (var evt in events)
          {
            if (evt.IsPoint)
              rawIntersections.Add(evt.ParameterA);
          }
        }
      }

      // If no intersection nodes were found on this curve, skip it.
      if (rawIntersections.Count == 0)
        continue;

      // 2. Build a complete, sorted list of parameter values.
      //    For open curves we add the endpoints so we know the full extent.
      List<double> allParams = new List<double>(rawIntersections);
      if (!crv.IsClosed)
      {
        allParams.Add(crv.Domain.Min);
        allParams.Add(crv.Domain.Max);
      }
      // Remove near-duplicates and sort the list.
      allParams = CleanParameters(crv, allParams, tolerance);
      // Also clean the raw intersections list.
      rawIntersections = CleanParameters(crv, rawIntersections, tolerance);

      // 3. For each parameter in our sorted list that is an intersection node,
      //    create two subcurves (if possible): one to the left (backward) and one to the right (forward).
      for (int i = 0; i < allParams.Count; i++)
      {
        double p = allParams[i];
        bool isIntersection = false;
        // Determine if this parameter comes from an actual intersection.
        foreach (double r in rawIntersections)
        {
          if (Math.Abs(p - r) < tolerance)
          {
            isIntersection = true;
            break;
          }
        }
        if (!isIntersection)
          continue; // skip parameters that are just endpoints

        // --- Create the left (backward) subcurve ---
        // If there is a previous parameter, then the interval [prev, p] is available.
        if (i > 0)
        {
          Interval leftInterval = new Interval(allParams[i - 1], p);
          double segLength = crv.GetLength(leftInterval);
          if (segLength > minLength)
          {
            // Extract the subcurve covering the full left interval.
            Curve leftSegment = crv.Trim(leftInterval.Min, leftInterval.Max);
            if (leftSegment != null && leftSegment.IsValid)
            {
              // We want a subcurve starting at the intersection node.
              // Reverse the left segment so that the intersection (p) becomes the start.
              Curve reversedLeft = leftSegment.DuplicateCurve();
              reversedLeft.Reverse();
              double targetLength = segLength * LengthFactor;
              double tReversed;
              if (reversedLeft.LengthParameter(targetLength, out tReversed))
              {
                Curve trimmedReversed = reversedLeft.Trim(reversedLeft.Domain.Min, tReversed);
                if (trimmedReversed != null && trimmedReversed.IsValid)
                {
                  // Reverse back to restore the original orientation.
                  trimmedReversed.Reverse();
                  resultSegments.Add(trimmedReversed);
                }
              }
            }
          }
        }

        // --- Create the right (forward) subcurve ---
        // If there is a next parameter, then the interval [p, next] is available.
        if (i < allParams.Count - 1)
        {
          Interval rightInterval = new Interval(p, allParams[i + 1]);
          double segLength = crv.GetLength(rightInterval);
          if (segLength > minLength)
          {
            Curve rightSegment = crv.Trim(rightInterval.Min, rightInterval.Max);
            if (rightSegment != null && rightSegment.IsValid)
            {
              double targetLength = segLength * LengthFactor;
              double tEnd;
              if (rightSegment.LengthParameter(targetLength, out tEnd))
              {
                Curve trimmed = rightSegment.Trim(rightSegment.Domain.Min, tEnd);
                if (trimmed != null && trimmed.IsValid)
                {
                  resultSegments.Add(trimmed);
                }
              }
            }
          }
        }
      }
    }

    OutputSegments = resultSegments;
  }

  /// <summary>
  /// Removes near-duplicate parameter values (within tolerance) and sorts them.
  /// Only parameters within the curve’s domain are kept.
  /// </summary>
  /// <param name="crv">The curve whose domain is considered.</param>
  /// <param name="parameters">List of parameter values.</param>
  /// <param name="tolerance">Tolerance for considering two parameters as duplicates.</param>
  /// <returns>A sorted list of unique parameter values.</returns>
  private List<double> CleanParameters(Curve crv, List<double> parameters, double tolerance)
  {
    List<double> cleaned = new List<double>();
    parameters.Sort();
    foreach (double t in parameters)
    {
      if (!crv.Domain.IncludesParameter(t))
        continue;
      bool duplicate = false;
      foreach (double exist in cleaned)
      {
        if (Math.Abs(t - exist) < tolerance)
        {
          duplicate = true;
          break;
        }
      }
      if (!duplicate)
        cleaned.Add(t);
    }
    return cleaned;
  }
}
