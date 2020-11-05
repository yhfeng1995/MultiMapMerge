/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include "Viewer.h"
#include <pangolin/pangolin.h>

#include <mutex>
#include <cmath>

namespace ORB_SLAM2
{

    Viewer::Viewer(System* pSystem, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Tracking *pTracking, const string &strSettingPath):
            mpSystem(pSystem), mpFrameDrawer(pFrameDrawer),mpMapDrawer(pMapDrawer), mpTracker(pTracking),
            mbFinishRequested(false), mbFinished(true), mbStopped(false), mbStopRequested(false)
    {
        cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

        float fps = fSettings["Camera.fps"];
        if(fps<1)
            fps=30;
        mT = 1e3/fps;

        mImageWidth = fSettings["Camera.width"];
        mImageHeight = fSettings["Camera.height"];
        if(mImageWidth<1 || mImageHeight<1)
        {
            mImageWidth = 640;
            mImageHeight = 480;
        }

        mViewpointX = fSettings["Viewer.ViewpointX"];
        mViewpointY = fSettings["Viewer.ViewpointY"];
        mViewpointZ = fSettings["Viewer.ViewpointZ"];
        mViewpointF = fSettings["Viewer.ViewpointF"];
    }

    void Viewer::Run()
    {
        mbFinished = false;

        pangolin::CreateWindowAndBind("ORB-SLAM2: Map Viewer",1024,768);

        // 3D Mouse handler requires depth testing to be enabled
        glEnable(GL_DEPTH_TEST);

        // Issue specific OpenGl we might need
        glEnable (GL_BLEND);
        glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(175));
        pangolin::Var<bool> menuFollowCamera("menu.Follow Camera",true,true);
        pangolin::Var<bool> menuShowPoints("menu.Show Points",true,true);
        pangolin::Var<bool> menuShowKeyFrames("menu.Show KeyFrames",true,true);
        pangolin::Var<bool> menuShowGraph("menu.Show Graph",true,true);
        pangolin::Var<bool> menuLocalizationMode("menu.Localization Mode",false,true);
        pangolin::Var<bool> menuReset("menu.Reset",false,false);
        //! Add for relocalization
        pangolin::Var<bool> menuSave("menu.Save",false,false);
        pangolin::Var<bool> menuLoad("menu.Load",false,false);
        pangolin::Var<bool> menuPause("menu.Pause",true,true);

        // 添加显示机器人位置选择项
        pangolin::Var<float> menuDisplayPose_x("menu.Pose:X",false,false);
        pangolin::Var<float> menuDisplayPose_Y("menu.Pose:Y",false,false);
        pangolin::Var<float> menuDisplayPose_Z("menu.Pose:Z",false,false);

        pangolin::Var<float> menuDisplayPose_Roll("menu.Pose:r",false,false);
        pangolin::Var<float> menuDisplayPose_Pitch("menu.Pose:p",false,false);
        pangolin::Var<float> menuDisplayPose_Yaw("menu.Pose:y",false,false);

        // Define Camera Render Object (for view / scene browsing)
        pangolin::OpenGlRenderState s_cam(
                pangolin::ProjectionMatrix(1024,768,mViewpointF,mViewpointF,512,389,0.1,1000),
                pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0)
        );

        // Add named OpenGL viewport to window and provide 3D Handler
        pangolin::View& d_cam = pangolin::CreateDisplay()
                .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f/768.0f)
                .SetHandler(new pangolin::Handler3D(s_cam));

        pangolin::OpenGlMatrix Twc;
        Twc.SetIdentity();

        cv::namedWindow("ORB-SLAM2: Current Frame");

        bool bFollow = true;
        bool bLocalizationMode = false;
        bool bPaused = true;
        bool bSave = false;
        bool bLoad = false;

        while(1)
        {
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            mpMapDrawer->GetCurrentOpenGLCameraMatrix(Twc);
            //cv::Mat Twc_dis = mpMapDrawer->returnCampose();

            {
                // 对界面显示的变量进行赋值
                menuDisplayPose_x = Twc.m[14];
                menuDisplayPose_Y = -Twc.m[12];
                menuDisplayPose_Z = -Twc.m[13];

                menuDisplayPose_Roll = atan(Twc.m[1]/Twc.m[0]) / 3.14 * 180;
                menuDisplayPose_Pitch = -atan(Twc.m[9] / Twc.m[10]) / 3.14 * 180;
                menuDisplayPose_Yaw = atan(-Twc.m[2] / sqrt( Twc.m[6] * Twc.m[6]+ Twc.m[10] * Twc.m[10])) / 3.14 * 180;
            }


            if(menuFollowCamera && bFollow)
            {
                s_cam.Follow(Twc);
            }
            else if(menuFollowCamera && !bFollow)
            {
                s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0));
                s_cam.Follow(Twc);
                bFollow = true;
            }
            else if(!menuFollowCamera && bFollow)
            {
                bFollow = false;
            }

            if(menuLocalizationMode && !bLocalizationMode)
            {
                mpSystem->ActivateLocalizationMode();
                bLocalizationMode = true;
            }
            else if(!menuLocalizationMode && bLocalizationMode)
            {
                mpSystem->DeactivateLocalizationMode();
                bLocalizationMode = false;
            }

            d_cam.Activate(s_cam);
            glClearColor(1.0f,1.0f,1.0f,1.0f);
            mpMapDrawer->DrawCurrentCamera(Twc);
            if(menuShowKeyFrames || menuShowGraph)
                mpMapDrawer->DrawKeyFrames(menuShowKeyFrames,menuShowGraph);
            if(menuShowPoints)
                mpMapDrawer->DrawMapPoints();

            pangolin::FinishFrame();

            cv::Mat im = mpFrameDrawer->DrawFrame();
            cv::imshow("ORB-SLAM2: Current Frame",im);
            cv::waitKey(mT);
            //! Pause the thread 暂停
            if(menuPause && !bPaused)
            {
                mpSystem->ActivatePause();
                bPaused = true;

                menuSave = false;
                menuLoad = false;
            }
            else if(!menuPause && bPaused)
            {
                mpSystem->ActivateUnPause();
                bPaused = false;

                menuSave = false;
                menuLoad = false;
            }

            //! save or load map when pasued
            if(bPaused)
            {
                //! check save
                if(menuSave && !bSave)
                {
                    mpSystem->ActivateSaveMap();//// mSaveFlag
                    bSave = true;
                }
                else if(!menuSave && bSave)
                {
                    bSave = false;
                }

                //! check load
                if(menuLoad && !bLoad)
                {
                    mpSystem->ActivateLoadMap();
                    bLoad = true;
                }
                else if(!menuLoad && bLoad)
                {
                    bLoad = false;
                }
            }

            if(menuReset && !bPaused)
            {
                menuShowGraph = true;
                menuShowKeyFrames = true;
                menuShowPoints = true;
                menuLocalizationMode = false;
                if(bLocalizationMode)
                    mpSystem->DeactivateLocalizationMode();
                bLocalizationMode = false;
                bFollow = true;
                menuFollowCamera = true;
                mpSystem->Reset();
                menuReset = false;

                menuPause = true;
                menuSave = false;
                menuLoad = false;
            }


            if(Stop())
            {
                while(isStopped())
                {
                    usleep(3000);
                }
            }

            if(CheckFinish())
                break;
        }

        SetFinish();
    }

    void Viewer::RequestFinish()
    {
        unique_lock<mutex> lock(mMutexFinish);
        mbFinishRequested = true;
    }

    bool Viewer::CheckFinish()
    {
        unique_lock<mutex> lock(mMutexFinish);
        return mbFinishRequested;
    }

    void Viewer::SetFinish()
    {
        unique_lock<mutex> lock(mMutexFinish);
        mbFinished = true;
    }

    bool Viewer::isFinished()
    {
        unique_lock<mutex> lock(mMutexFinish);
        return mbFinished;
    }

    void Viewer::RequestStop()
    {
        unique_lock<mutex> lock(mMutexStop);
        if(!mbStopped)
            mbStopRequested = true;
    }

    bool Viewer::isStopped()
    {
        unique_lock<mutex> lock(mMutexStop);
        return mbStopped;
    }

    bool Viewer::Stop()
    {
        unique_lock<mutex> lock(mMutexStop);
        unique_lock<mutex> lock2(mMutexFinish);

        if(mbFinishRequested)
            return false;
        else if(mbStopRequested)
        {
            mbStopped = true;
            mbStopRequested = false;
            return true;
        }

        return false;

    }

    void Viewer::Release()
    {
        unique_lock<mutex> lock(mMutexStop);
        mbStopped = false;
    }

}
