/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Eitan Marder-Eppstein
*********************************************************************/
#include <tf2_ros/buffer_server.h>
namespace tf2_ros
{
  BufferServer::BufferServer(rclcpp::Node::SharedPtr node,const Buffer& buffer, const std::string& ns, bool auto_start, tf2::Duration check_period):
    buffer_(buffer),
    node_(node)
  {
    server_ = rclcpp_action::create_server<tf2_msgs::action::LookupTransform>(
                    node_,
                    ns,
                    std::bind(&BufferServer::handleGoalCB, this, std::placeholders::_1, std::placeholders::_2),
                    std::bind(&BufferServer::cancelCB, this, std::placeholders::_1),
                    std::bind(&BufferServer::acceptGoalCB, this, std::placeholders::_1));

    check_timer_ = node_->create_wall_timer(check_period, std::bind(&BufferServer::checkTransforms, this));
  }

  void BufferServer::checkTransforms()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    for(std::list<GoalInfo>::iterator it = active_goals_.begin(); it != active_goals_.end();)
    {
      GoalInfo& info = *it;

      //we want to lookup a transform if the time on the goal
      //has expired, or a transform is available
      if(canTransform(info.handle) || info.end_time < tf2::get_now())
      {
        tf2_msgs::action::LookupTransform::Result::SharedPtr result;

        //try to populate the result, catching exceptions if they occur
        try
        {
          result->transform = lookupTransform(info.handle);
        }
        catch (tf2::ConnectivityException &ex)
        {
          result->error.error = result->error.CONNECTIVITY_ERROR;
          result->error.error_string = ex.what();
        }
        catch (tf2::LookupException &ex)
        {
          result->error.error = result->error.LOOKUP_ERROR;
          result->error.error_string = ex.what();
        }
        catch (tf2::ExtrapolationException &ex)
        {
          result->error.error = result->error.EXTRAPOLATION_ERROR;
          result->error.error_string = ex.what();
        }
        catch (tf2::InvalidArgumentException &ex)
        {
          result->error.error = result->error.INVALID_ARGUMENT_ERROR;
          result->error.error_string = ex.what();
        }
        catch (tf2::TimeoutException &ex)
        {
          result->error.error = result->error.TIMEOUT_ERROR;
          result->error.error_string = ex.what();
        }
        catch (tf2::TransformException &ex)
        {
          result->error.error = result->error.TRANSFORM_ERROR;
          result->error.error_string = ex.what();
        }

        //make sure to pass the result to the client
        //even failed transforms are considered a success
        //since the request was successfully processed
        it = active_goals_.erase(it);
        info.handle->succeed(result);
      }
      else
        ++it;
    }
  }

  rclcpp_action::CancelResponse BufferServer::cancelCB(const std::shared_ptr<rclcpp_action::ServerGoalHandle<tf2_msgs::action::LookupTransform>> gh)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    //we need to find the goal in the list and remove it... also setting it as canceled
    //if its not in the list, we won't do anything since it will have already been set
    //as completed
    tf2_msgs::action::LookupTransform::Result::SharedPtr result;
    for(std::list<GoalInfo>::iterator it = active_goals_.begin(); it != active_goals_.end();)
    {
      GoalInfo& info = *it;
      if(info.handle == gh)
      {
        it = active_goals_.erase(it);
        info.handle->canceled(result);
        return rclcpp_action::CancelResponse::ACCEPT;
      }
      else
        ++it;
    }
  }
  rclcpp_action::GoalResponse BufferServer::handleGoalCB(const rclcpp_action::GoalUUID & uuid,
                                                         std::shared_ptr<const tf2_msgs::action::LookupTransform::Goal> gh)
  {
    (void)uuid;
    // TODO (anasarrak): Add a REJECT
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }
  void BufferServer::acceptGoalCB(const std::shared_ptr<rclcpp_action::ServerGoalHandle<tf2_msgs::action::LookupTransform>> gh)
  {
    std::thread(&BufferServer::goalCB, this, gh).detach();
  }
  void BufferServer::goalCB(const std::shared_ptr<rclcpp_action::ServerGoalHandle<tf2_msgs::action::LookupTransform>> gh)
  {
    //we'll accept all goals we get
    // gh.setAccepted();

    //if the transform isn't immediately available, we'll push it onto our list to check
    //along with the time that the goal will end
    GoalInfo goal_info;
    goal_info.handle = gh;
    goal_info.end_time = tf2::get_now() + tf2::durationFromSec(gh->get_goal()->timeout.sec);

    //we can do a quick check here to see if the transform is valid
    //we'll also do this if the end time has been reached
    if(canTransform(gh) || goal_info.end_time <= tf2::get_now())
    {
      tf2_msgs::action::LookupTransform::Result::SharedPtr result;
      try
      {
        result->transform = lookupTransform(gh);
      }
      catch (tf2::ConnectivityException &ex)
      {
        result->error.error = result->error.CONNECTIVITY_ERROR;
        result->error.error_string = ex.what();
      }
      catch (tf2::LookupException &ex)
      {
        result->error.error = result->error.LOOKUP_ERROR;
        result->error.error_string = ex.what();
      }
      catch (tf2::ExtrapolationException &ex)
      {
        result->error.error = result->error.EXTRAPOLATION_ERROR;
        result->error.error_string = ex.what();
      }
      catch (tf2::InvalidArgumentException &ex)
      {
        result->error.error = result->error.INVALID_ARGUMENT_ERROR;
        result->error.error_string = ex.what();
      }
      catch (tf2::TimeoutException &ex)
      {
        result->error.error = result->error.TIMEOUT_ERROR;
        result->error.error_string = ex.what();
      }
      catch (tf2::TransformException &ex)
      {
        result->error.error = result->error.TRANSFORM_ERROR;
        result->error.error_string = ex.what();
      }

      gh->succeed(result);
      return;
    }

    std::unique_lock<std::mutex> lock(mutex_);
    active_goals_.push_back(goal_info);
  }

  bool BufferServer::canTransform(std::shared_ptr<rclcpp_action::ServerGoalHandle<tf2_msgs::action::LookupTransform>> gh)
  {
    const std::shared_ptr<const tf2_msgs::action::LookupTransform::Goal>& goal = gh->get_goal();

    //check whether we need to used the advanced or simple api
    if(!goal->advanced)
      return buffer_.canTransform(goal->target_frame, goal->source_frame,tf2::timeFromSec(goal->source_time.sec));

    return buffer_.canTransform(goal->target_frame, tf2::timeFromSec(goal->target_time.sec),
        goal->source_frame, tf2::timeFromSec(goal->source_time.sec), goal->fixed_frame);
  }

  geometry_msgs::msg::TransformStamped BufferServer::lookupTransform(std::shared_ptr<rclcpp_action::ServerGoalHandle<tf2_msgs::action::LookupTransform>> gh)
  {
    const std::shared_ptr<const tf2_msgs::action::LookupTransform::Goal>& goal = gh->get_goal();

    //check whether we need to used the advanced or simple api
    if(!goal->advanced)
      return buffer_.lookupTransform(goal->target_frame, goal->source_frame, tf2::timeFromSec(goal->source_time.sec));

    return buffer_.lookupTransform(goal->target_frame, tf2::timeFromSec(goal->target_time.sec),
        goal->source_frame, tf2::timeFromSec(goal->source_time.sec), goal->fixed_frame);
  }

  void BufferServer::start()
  {
    // server_.start();
  }

};
