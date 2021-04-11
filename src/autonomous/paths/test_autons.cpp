#include "main.h"

#include "api.h"
#include <math.h>
#include "globals.hpp"
#include "autonomous/auton_control.hpp"
#include "autonomous/auto_timer.hpp"
#include "autonomous/auto_sequence.hpp"
#include <vector>
#include <functional>
#include "autonomous/auto_drive.hpp"
#include "autonomous/global_sequences.hpp"
#include "autonomous.h"

using namespace std;
using namespace Auton;
using namespace pros;

AutoSequence *Auton::AT_Test_Ultras = AutoSequence::FromTasks(
    vector<AutoTask>{
        //     each tile is 24 inches, (0,0) at center of field, width of bot is 18, length is 14, tracked at center of bot, max distance is 3 tiles (72).
        // autopath(AUTO_DRIVE.CPP) drives to a certain point P {0, -72}, and it will have the angle 0, and reach that point of 127

        SingleRun([](void) -> void {
            robotPos.x = 36;
            robotPos.y = 12;
            robotPos.angle = 0;
        }),
        AutoTask::AutoDelay(500),
        PurePursuitTask(36, 30, 2.5, 8, true), //36, 36
        TurnToPointTask(12, 15, 0.1),          //13, 16
        PurePursuitTask(12, 15, 3, 8, true),
        ApproachGoalTask(50, 650),
        AutoTask::AutoDelay(1000),
        ApproachGoalTask(-70, 500),
        AutoTask::AutoDelay(600),
        TurnToPointTask(26, 74, 0.1),
        PurePursuitTask(26, 74, 2.5, 8, true),
        TurnToPointTask(17, 75, 0.1),
        PurePursuitTask(17, 75, 2.5, 8, true),
        ApproachGoalTask(50, 650),
        AutoTask::AutoDelay(1000),
        ApproachGoalTask(-65, 650),
        
        // SingleRun([](void) -> void { position_tracker->Set_Position({0, 0}, 0, {50, 1}, 0); }),
    });

// AutoSequence *Auton::AT_Test_Ultras = AutoSequence::FromTasks(
//     vector<AutoTask>{
//         SingleRun([](void) -> void { position_tracker->Set_Position({48, 0}, 0); }),

//         AutoPath({58, -30}, -M_PI / 4, 160, 3).AddRun([](void) -> void {
//                 intake->Set_Intake(127);
//                 shooter->Set_Shooter(0);
//                 indexer->Set_Indexer(100, true);
//         }),
//         AutoTask::SyncTask(
//             [](void) -> void {
//                     robot->drive->Set_Curve_Drive({54, -53}, -M_PI / 4 + 0.05, {58, -56}, -M_PI / 4 + 0.05, 160, 0.5);
//                     intake->Set_Intake(127);
//                     indexer->Set_Indexer(100, true);
//             },
//             [](void) -> bool { return (!robot->drive->get_running()); }, [](void) -> void { robot->drive->Reset_Point(); }, [](void) -> void {}),

//         AutoPath({58, -56}, -M_PI / 4 + 0.05, 160).AddRun([](void) -> void {
//                 intake->Set_Intake(0);
//                 shooter->Set_Shooter(0);
//                 indexer->Set_Indexer(100, true);
//         }),
//         AutoTask::AutoDelay(500).AddRun([](void) -> void {
//                                         intake->Set_Intake(0);
//                                         shooter->Shoot(127);
//                                 })
//             .AddInit([](void) -> void { indexer->resetNewBall(); })
//             .AddDone([](void) -> bool { return indexer->newBallIndexed(); }),

//         AutoTask::SyncTask(
//             [](void) -> void {
//                     robot->drive->Set_Curve_Drive({60, -53}, -M_PI / 4 + 0.15, {56, -46}, -M_PI, 127, 3);
//                     intake->Set_Intake(-30);
//                     shooter->Set_Shooter(0);
//                     indexer->Set_Indexer(100, true);
//             },
//             [](void) -> bool { return (!robot->drive->get_running()); }, [](void) -> void { robot->drive->Reset_Point(); }, [](void) -> void {}),

//         AutoPath({56, -46}, -M_PI, 180, {3, 5}).AddRun([](void) -> void {
//                 intake->Set_Intake(0);
//                 indexer->Set_Indexer(0);
//         }),
//         AutoTask::SyncTask(
//             [](void) -> void {
//                     robot->drive->Set_Curve_Drive({28, -46}, -M_PI, {0, -24}, -5 * M_PI / 4, 150, 3);
//                     intake->Set_Intake(127);
//                     shooter->Set_Shooter(0);
//                     indexer->Set_Indexer(100, true);
//             },
//             [](void) -> bool { return (!robot->drive->get_running()); }, [](void) -> void { robot->drive->Reset_Point(); }, [](void) -> void {}),

//         AutoPath({0, -28}, -10 * M_PI / 8, {127, 150}, 2).AddRun([](void) -> void {
//                 intake->Set_Intake(127);
//                 indexer->Set_Indexer(127, true);
//         }),
//         AutoPath({2, -28}, -M_PI / 2, {127, 180}, 1),

//         AutoPath({2, -56}, -M_PI / 2, {150, 150}, 1),

//         AutoTask::AutoDelay(1000).AddRun([](void) -> void {
//                                          intake->Set_Intake(0);
//                                          shooter->Shoot(127);
//                                          robot->drive->Set_Path_Drive({2, -56}, -M_PI / 2, 80);
//                                  })
//             .AddInit([](void) -> void {
//                     indexer->resetNewBall();
//                     robot->drive->Reset_Point();
//             })
//             .AddDone([](void) -> bool { return indexer->newBallIndexed(); }),
//         AutoTask::AutoDelay(500).AddRun([](void) -> void {
//                 indexer->Set_Indexer(70, false);
//                 shooter->Shoot(0);
//                 robot->drive->Set_Path_Drive({2, -56}, -M_PI / 2, 80);
//         }),
//         AutoTask::AutoDelay(700).AddRun([](void) -> void {
//                                         intake->Set_Intake(0);
//                                         shooter->Shoot(127);
//                                 })
//             .AddKill([](void) -> void { shooter->Shoot(0); indexer->Set_Indexer(0); })
//             .AddInit([](void) -> void { indexer->resetNewBall(); })
//             .AddDone([](void) -> bool { return indexer->newBallIndexed(); }),
//         AutoTask::AutoDelay(10000000),
//     });
