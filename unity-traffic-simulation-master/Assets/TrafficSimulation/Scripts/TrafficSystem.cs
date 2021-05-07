using System.Collections.Generic;
using UnityEngine;

namespace TrafficSimulation {
    public class TrafficSystem : MonoBehaviour {
        public bool hideGuizmos = false;
        public float segDetectThresh = 0.1f;
        public ArrowDraw arrowDrawType = ArrowDraw.ByLength;
        public int arrowCount = 1;
        public float arrowDistance = 5;
        public float arrowSizeWaypoint = 1;
        public float arrowSizeIntersection = 0.5f;
        public float waypointSize = 0.5f;
        public string[] collisionLayers;
        
        public List<Segment> segments = new List<Segment>();
        public List<Intersection> intersections = new List<Intersection>();
        public Segment curSegment = null;  
        
        public List<Waypoint> GetAllWaypoints() {
            List<Waypoint> points = new List<Waypoint>();

            foreach (Segment segment in segments) {
                points.AddRange(segment.waypoints);
            }

            return points;
        }

        public void SaveTrafficSystem(){
            Intersection[] intersections = GameObject.FindObjectsOfType<Intersection>();
            foreach(Intersection intersection in intersections)
                intersection.SaveIntersectionStatus();
        }

        public void ResumeTrafficSystem(){
            Intersection[] intersections = GameObject.FindObjectsOfType<Intersection>();
            foreach(Intersection intersection in intersections)
                intersection.ResumeIntersectionStatus();
        }
    }

    public enum ArrowDraw {
        FixedCount, ByLength, Off
    }
}
