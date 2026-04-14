# System Design Document

## Purpose
This document defines the system level architecture for the Autonomous Racing Prototype (ARP) project, including system components, interfaces, and design decisions. Low-level design elements will not be described in detail in this document for the sake of clear communication. The goal of this document is to streamline the development process of project-ARP through clear documentation. This document is not a reference for outside persons. It is solely intended for internal documentation and tracking.

## Constraints and Objectives
- An Autonomous RC car that is faster around a track than the best effort of a human driver.
- Material cost < $2000
- Racecar/sports car in appearance
- Top speed of at least 50 mph
- Aerodynamic downforce of at least a factor of 2
- Controllable (Stable enough that a human could control the car)
- 10 minute runtime minimum at full charge
- Durable enough to survive several crashes (crashes with cones or driving off track and tumbling)
- Less than 5% communication downtime between the car and the computer
- Less than 5% crash rate
- Not a serious fire hazard

## System Level Architecture

## Interfaces

## Hardware Architecture

## Electronics Architecture

## Software Architecture

### Computer Vision
The computer vision pipeline is arguably the most important software module in this project and corrospondingly requires careful consideration. The main problems the computer vision system needs to overcome are fast object dettection, occlusion, object localization, multi object detection, and object tracking. There are many possible algorithms but the most popular are neural network based like R-CNN and YOLO. CNN's are the prototypical examples but they require multiple loops to detect multiple objects as opposed to YOLO which only needs a single pass to detect objects. Due to the single pass nature of YOLO it has become an industry standard for real time object detection. 
A YOLO model will be trained and used for project-ARP. Video should be taken of the track through the lens of the onboard camera on the robot and labeled through a data labeling software like [Label Studio](https://labelstud.io). Because manual data labeling is tedious, labeled data will be augmented by some custom scripts to digitally rotate images, change lighting, add differetn colored lights, etc. This data augmentations has been shown to improve model accuracy for little effort.
Object tracking TODO

### SLAM
During racing laps, the object detection algorithm will detect the first 2-3 cones in front of the car and perform linear interpolation between these points. These lines will then be fitted to the current model of the track to give an estimate of the cars pose.

## Open Questions / Risks

## Development Plan
These phases are meant as dateable milestones for project management purposes to ensure timely completion of the project. This separation of phases is not meant to discourage cross-phase work. Develepment, integration, and testing should always be in mind during R & D. Likewise, if an issue arises during integration or development the engineer should not hesitate to go back to the drawing board to reevaluate role and function of problematic design decisions in the project.

### Phase 1: Research and Design

### Phase 2: Development

### Phase 3: Integration Hell

### Phase 4: Testing
Final System Test Lap function
1. Learning Lap
2. Stop-Start Lap
3. Three Running Start Laps

### Phase 5: Final Write-Up and Catch-Up on Sleep

## Meeting Notes
**April 14, 2026**
Discussed System Design Document

Action Items
- [ ] Prelimanary thermodynamic analysis @JosephPennock
- [ ] Prelimanary Object tracking design @NathanNeidigh


**March 31, 2026**
Delegation of initial tasks and institution of the late work slap penalty.

Action Items
- [X] Prelimanary computer vision design @NathanNeidigh
- [ ] Hardware inventory @SethWaller
- [X] Prelimanary power delivery system design @JosephPennock
