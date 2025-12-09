# Tasks: AI/Spec-Driven Book — Physical AI, Humanoid Robotics & Agentic Engineering

**Input**: Design documents from `/specs/1-humanoid-robotics-book/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: The feature specification does not explicitly request test tasks to be created in the task list; however, the plan mentions "reproducible tests" and "simulation validation." Therefore, I will include tasks for creating reproducible tests within each module's lab directory.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Book content**: `book/docs/`, `book/static/`, `book/src/`
- **Labs**: `labs/moduleX-name/`
- Paths shown below align with the dual-root structure defined in `plan.md`

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure for the book and labs.

- [x] T001 Create book content directory structure `my-humanoid-book/docs/` `my-humanoid-book/static/` `my-humanoid-book/src/`
- [x] T002 Initialize Docusaurus project in `my-humanoid-book/`
- [x] T003 Configure Docusaurus for GitHub Pages deployment `my-humanoid-book/docusaurus.config.js`
- [x] T004 Set up initial markdown structure for book chapters `my-humanoid-book/docs/`
- [x] T005 Create labs root directory `labs/`
- [x] T006 Initialize Python environment for labs in `.`

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure and tools that MUST be complete before ANY user story can be implemented.

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [x] T007 Configure git for the project (e.g., .gitignore, LFS if needed) `.`
- [x] T008 Ensure Claude Code setup for project assistance `CLAUDE.md`

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel.

---

## Phase 3: User Story 1 - Develop Robotic Nervous System (ROS 2) (Priority: P1) 🎯 MVP

**Goal**: Learn and implement ROS 2 for humanoid robot control to build the foundational communication layer.

**Independent Test**: Can be fully tested by creating and running a basic ROS2 humanoid control package that demonstrates basic communication (nodes, topics, actions).

### Implementation for User Story 1

- [x] T009 [US1] Create module directory `labs/module1-ros2/`
- [x] T010 [US1] Set up ROS2 workspace in `labs/module1-ros2/ros2_ws/src/`
- [x] T011 [US1] Create basic ROS2 package for humanoid control `labs/module1-ros2/ros2_ws/src/humanoid_control_pkg/`
- [x] T012 [P] [US1] Implement ROS2 nodes for basic communication in `labs/module1-ros2/ros2_ws/src/humanoid_control_pkg/src/`
- [x] T013 [P] [US1] Integrate `rclpy` for Python-based ROS2 nodes `labs/module1-ros2/ros2_ws/src/humanoid_control_pkg/src/`
- [x] T014 [US1] Develop URDF model for humanoid robot in `labs/module1-ros2/ros2_ws/src/humanoid_description/`
- [x] T015 [P] [US1] Implement basic motor drivers (simulated) in `labs/module1-ros2/ros2_ws/src/humanoid_control_pkg/src/`
- [x] T016 [US1] Add reproducible tests for ROS2 package in `labs/module1-ros2/ros2_ws/src/humanoid_control_pkg/test/`
- [x] T017 [US1] Document week 3-5 chapters for ROS2 Nervous System `book/docs/`

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently.

---

## Phase 4: User Story 2 - Create Digital Twin (Gazebo & Unity) (Priority: P1)

**Goal**: Build and simulate a digital twin of a humanoid robot in Gazebo/Unity.

**Independent Test**: Can be fully tested by successfully launching, interacting with, and observing realistic behavior of a simulated humanoid twin within Gazebo or Unity.

### Implementation for User Story 2

- [x] T018 [US2] Create module directory `labs/module2-digital-twin/`
- [x] T019 [US2] Set up Gazebo environment for humanoid simulation in `labs/module2-digital-twin/gazebo_sim/`
- [x] T020 [US2] Integrate URDF model into Gazebo simulation `labs/module2-digital-twin/gazebo_sim/`
- [x] T021 [P] [US2] Implement physics and collision detection in Gazebo `labs/module2-digital-twin/gazebo_sim/`
- [x] T022 [P] [US2] Simulate basic sensors (e.g., IMU, joint encoders) in Gazebo `labs/module2-digital-twin/gazebo_sim/`
- [x] T023 [US2] Set up Unity project for visualization and human-robot interaction in `labs/module2-digital-twin/unity_hri/`
- [x] T024 [US2] Connect Unity visualization to Gazebo simulation (if applicable) `labs/module2-digital-twin/unity_hri/`
- [x] T025 [US2] Add reproducible tests for digital twin in `labs/module2-digital-twin/test/`
- [x] T026 [US2] Document week 6-8 chapters for Digital Twin `book/docs/`

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently.

---

## Phase 5: User Story 3 - Build AI Robot Brain (NVIDIA Isaac) (Priority: P2)

**Goal**: Integrate NVIDIA Isaac for SLAM, navigation, and perception.

**Independent Test**: Can be fully tested by demonstrating the simulated humanoid's ability to map its environment (SLAM), navigate to target locations, and identify objects (perception) using NVIDIA Isaac components.

### Implementation for User Story 3

- [x] T027 [US3] Create module directory `labs/module3-isaac-brain/`
- [x] T028 [US3] Set up NVIDIA Isaac Sim environment for synthetic data generation `labs/module3-isaac-brain/isaac_sim/`
- [x] T029 [US3] Generate synthetic sensor data for SLAM and perception `labs/module3-isaac-brain/isaac_sim/`
- [x] T030 [P] [US3] Integrate Isaac ROS for SLAM capabilities in `labs/module3-isaac-brain/isaac_ros_ws/`
- [x] T031 [P] [US3] Implement navigation stack for autonomous movement in `labs/module3-isaac-brain/isaac_ros_ws/`
- [x] T032 [P] [US3] Develop perception modules for object detection/recognition in `labs/module3-isaac-brain/isaac_ros_ws/`
- [x] T033 [US3] Add reproducible tests for Isaac AI brain in `labs/module3-isaac-brain/test/`
- [x] T034 [US3] Document week 9-11 chapters for NVIDIA Isaac Brain `book/docs/`

**Checkpoint**: All user stories up to this point should now be independently functional.

---

## Phase 6: User Story 4 - Develop Vision-Language-Action Robotics (VLA) (Priority: P2)

**Goal**: Implement Vision-Language-Action (VLA) robotics for voice-commanded control.

**Independent Test**: Can be fully tested by issuing a voice command to the simulated robot and observing its ability to interpret the command, plan a sequence of actions, and execute those actions through motor control.

### Implementation for User Story 4

- [x] T035 [US4] Create module directory `labs/module4-vla-robotics/`
- [x] T036 [US4] Integrate voice command interface (e.g., Whisper) in `labs/module4-vla-robotics/vla_agent/src/`
- [x] T037 [P] [US4] Develop LLM-based planning module for action sequencing in `labs/module4-vla-robotics/vla_agent/src/`
- [x] T038 [P] [US4] Implement action execution and motor control interface in `labs/module4-vla-robotics/vla_agent/src/`
- [x] T039 [US4] Connect VLA agent to simulated humanoid for autonomous actions `labs/module4-vla-robotics/vla_agent/`
- [x] T040 [US4] Add reproducible tests for VLA agent in `labs/module4-vla-robotics/test/`
- [x] T041 [US4] Document week 12-13 chapters for Vision-Language-Action Robotics `book/docs/`

**Checkpoint**: All user stories should now be independently functional.

---

## Phase 7: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories and final book preparation.

- [x] T042 [P] Review and refine Docusaurus content and navigation `book/docs/`
- [x] T043 Optimize build scripts and configurations across all labs and book `.`
- [x] T044 Implement Jetson edge deployment examples for labs `labs/`
- [x] T045 [P] Add challenges and extensions for each module `book/docs/`
- [x] T046 Ensure all labs are independently repeatable and testable `labs/`
- [x] T047 Final end-to-end integration and capstone blueprint documentation `book/docs/`

---

## Dependencies & Execution Order

### Phase Dependencies

-   **Setup (Phase 1)**: No dependencies - can start immediately.
-   **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories.
-   **User Stories (Phase 3+)**: All depend on Foundational phase completion.
    -   User stories can then proceed in parallel (if staffed).
    -   Or sequentially in priority order (P1 → P2 → P3).
-   **Polish (Final Phase)**: Depends on all desired user stories being complete.

### User Story Dependencies

-   **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories.
-   **User Story 2 (P1)**: Can start after Foundational (Phase 2) - May integrate with US1 but should be independently testable.
-   **User Story 3 (P2)**: Can start after Foundational (Phase 2) - May integrate with US1/US2 but should be independently testable.
-   **User Story 4 (P2)**: Can start after Foundational (Phase 2) - May integrate with US1/US2/US3 but should be independently testable.

### Within Each User Story

-   Tests (if included) MUST be written and FAIL before implementation.
-   Models before services.
-   Services before endpoints.
-   Core implementation before integration.
-   Story complete before moving to next priority.

### Parallel Opportunities

-   All Setup tasks marked [P] can run in parallel.
-   All Foundational tasks marked [P] can run in parallel (within Phase 2).
-   Once Foundational phase completes, User Story 1 and User Story 2 can start in parallel (if team capacity allows).
-   Tasks T012, T013, T015 within User Story 1 can run in parallel.
-   Tasks T021, T022 within User Story 2 can run in parallel.
-   Tasks T030, T031, T032 within User Story 3 can run in parallel.
-   Tasks T037, T038 within User Story 4 can run in parallel.
-   Tasks T042, T045 within Polish & Cross-Cutting Concerns can run in parallel.

---

## Parallel Example: User Story 1

```bash
# Launch all parallelizable implementation tasks for User Story 1 together:
Task: "Implement ROS2 nodes for basic communication in labs/module1-ros2/ros2_ws/src/humanoid_control_pkg/src/"
Task: "Integrate rclpy for Python-based ROS2 nodes labs/module1-ros2/ros2_ws/src/humanoid_control_pkg/src/"
Task: "Implement basic motor drivers (simulated) in labs/module1-ros2/ros2_ws/src/humanoid_control_pkg/src/"
```

---

## Implementation Strategy

### MVP First (User Story 1 & 2 Only)

1.  Complete Phase 1: Setup
2.  Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3.  Complete Phase 3: User Story 1
4.  Complete Phase 4: User Story 2
5.  **STOP and VALIDATE**: Test User Story 1 and User Story 2 independently
6.  Deploy/demo if ready

### Incremental Delivery

1.  Complete Setup + Foundational → Foundation ready
2.  Add User Story 1 → Test independently → Deploy/Demo (MVP!)
3.  Add User Story 2 → Test independently → Deploy/Demo
4.  Add User Story 3 → Test independently → Deploy/Demo
5.  Add User Story 4 → Test independently → Deploy/Demo
6.  Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1.  Team completes Setup + Foundational together
2.  Once Foundational is done:
    -   Developer A: User Story 1
    -   Developer B: User Story 2
    -   Developer C: User Story 3
    -   Developer D: User Story 4
3.  Stories complete and integrate independently

---

## Notes

-   [P] tasks = different files, no dependencies
-   [Story] label maps task to specific user story for traceability
-   Each user story should be independently completable and testable
-   Verify tests fail before implementing
-   Commit after each task or logical group
-   Stop at any checkpoint to validate story independently
-   Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence
