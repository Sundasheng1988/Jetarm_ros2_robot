# Robot Runtime — Session Start

Read in order:

1. docs/runtime_index.md
2. docs/runtime_session_summary.md
3. docs/topic_service_map.md

Current Phase

Robot Runtime Integration

Current Sprint

Sprint 5.6

Goal

Switch:

/world_model/stable_objects
↓

grounding_node

Do NOT modify:

- servo_controller
- kinematics
- ros_robot_controller
- hardware SDK

Current Rules

YOLO
→ semantic authority

ROI
→ pose + color candidates

Fusion
→ perception_objects

StableTracker
→ stable_objects

Grounding
→ consume stable_objects

ROI color:

low confidence
↓

unknown