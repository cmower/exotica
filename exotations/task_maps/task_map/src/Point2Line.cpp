/*
 *      Author: Christian Rauch
 *
 * Copyright (c) 2018, University Of Edinburgh
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  * Neither the name of  nor the names of its contributors may be used to
 *    endorse or promote products derived from this software without specific
 *    prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "task_map/Point2Line.hpp"

REGISTER_TASKMAP_TYPE("Point2Line", exotica::Point2Line);

namespace exotica
{
Eigen::Vector3d Point2Line::direction(const Eigen::Vector3d &point, const Eigen::Vector3d &line, const bool infinite, const bool dbg)
{
    // http://mathworld.wolfram.com/Point-LineDistance3-Dimensional.html
    // let:
    //      s: start of line
    //      e: end of line
    //      p: point
    // then the point on vector v = e-s that is closest to p is vp = s + t*(e-s) with
    // t = ((s-p)*(e-s)) / (|e-s|^2)    (* denotes the dot product)

    // the line starts at the origin of BaseOffset, hence s=0, v='line', vp=t*'line'
    double t = (point).dot(line) / line.norm();
    if (dbg) HIGHLIGHT_NAMED("P2L", "t " << t);
    if (!infinite)
    {
        // clip to to range [0,1]
        t = std::min(std::max(0.0, t), 1.0);
        if (dbg) HIGHLIGHT_NAMED("P2L", "|t| " << t);
    }

    // vector from point 'p' to point 'vp' on line
    const Eigen::Vector3d dv = (t * line) - point;
    if (dbg) HIGHLIGHT_NAMED("P2L", "dv " << dv.transpose());
    return dv;
}

void Point2Line::update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi)
{
    if (phi.rows() != Kinematics[0].Phi.rows()) throw_named("Wrong size of phi!");

    for (int i = 0; i < Kinematics[0].Phi.rows(); i++)
    {
        phi(i) = direction(Eigen::Map<const Eigen::Vector3d>(Kinematics[0].Phi(i).p.data), line, infinite, debug_).norm();
    }
}

void Point2Line::update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi, Eigen::MatrixXdRef J)
{
    if (phi.rows() != Kinematics[0].Phi.rows()) throw_named("Wrong size of phi!");
    if (J.rows() != Kinematics[0].J.rows() || J.cols() != Kinematics[0].J(0).data.cols()) throw_named("Wrong size of J! " << Kinematics[0].J(0).data.cols());

    for (int i = 0; i < Kinematics[0].Phi.rows(); i++)
    {
        // direction from point to line
        const Eigen::Vector3d dv = direction(Eigen::Map<const Eigen::Vector3d>(Kinematics[0].Phi(i).p.data), line, infinite, debug_);
        phi(i) = dv.norm();
        for (int j = 0; j < J.cols(); j++)
        {
            if (phi(i) > 1e-9)
            {
                J(i, j) = -(dv.normalized()).dot(Eigen::Map<const Eigen::Vector3d>(Kinematics[0].J[i].getColumn(j).vel.data));
            }
        }

        // visualisation of point, line and their distance
        if (debug_ && Server::isRos())
        {
            visualization_msgs::MarkerArray ma;
            const Eigen::Map<const Eigen::Vector3d> p(Kinematics[0].Phi(i).p.data);
            const ros::Time t = ros::Time::now();
            const std::string common_frame = "exotica/" + base_name;
            {
                // line in base frame
                visualization_msgs::Marker mc;
                mc.header.stamp = t;
                mc.frame_locked = true;
                mc.header.frame_id = common_frame;
                mc.ns = "cam/line/" + object_name_;
                mc.type = visualization_msgs::Marker::ARROW;
                mc.scale.x = 0.01;
                mc.scale.y = 0.01;
                mc.scale.z = 0.01;
                // line start
                geometry_msgs::Point p;
                p.x = line_start.x();
                p.y = line_start.y();
                p.z = line_start.z();
                mc.points.push_back(p);
                // line end
                const Eigen::Vector3d pe = line_end;
                p.x = pe.x();
                p.y = pe.y();
                p.z = pe.z();
                mc.points.push_back(p);
                mc.color.r = 1;
                mc.color.g = 1;
                mc.color.b = 0;
                mc.color.a = 0.2;
                ma.markers.push_back(mc);
            }
            {
                // point in link frame
                visualization_msgs::Marker ml;
                ml.header.stamp = t;
                ml.frame_locked = true;
                ml.header.frame_id = common_frame;
                ml.ns = "lnk/point/" + object_name_;
                ml.type = visualization_msgs::Marker::SPHERE;
                ml.scale.x = 0.03;
                ml.scale.y = 0.03;
                ml.scale.z = 0.03;
                ml.color.r = 1;
                ml.color.g = 0;
                ml.color.b = 0;
                ml.color.a = 0.2;
                ml.pose.position.x = p.x();
                ml.pose.position.y = p.y();
                ml.pose.position.z = p.z();
                ma.markers.push_back(ml);
            }
            {
                // draw 'dv' starting at 'p' in base frame
                visualization_msgs::Marker mdv;
                mdv.header.stamp = t;
                mdv.frame_locked = true;
                mdv.header.frame_id = common_frame;
                mdv.ns = "dv/" + object_name_;
                mdv.type = visualization_msgs::Marker::ARROW;
                mdv.scale.x = 0.001;
                mdv.scale.y = 0.01;
                mdv.scale.z = 0.01;
                mdv.pose.position.x = p.x();
                mdv.pose.position.y = p.y();
                mdv.pose.position.z = p.z();
                mdv.points.push_back(geometry_msgs::Point());
                geometry_msgs::Point pdv;
                pdv.x = dv.x();
                pdv.y = dv.y();
                pdv.z = dv.z();
                mdv.points.push_back(pdv);
                mdv.color.r = 0;
                mdv.color.g = 1;
                mdv.color.b = 0;
                mdv.color.a = 0.5;
                ma.markers.push_back(mdv);
            }
            pub_marker.publish(ma);
        }
    }
}

void Point2Line::Instantiate(Point2LineInitializer &init)
{
    link_name = boost::any_cast<std::string>(init.EndEffector[0].getProperty("Link"));
    base_name = boost::any_cast<std::string>(init.EndEffector[0].getProperty("Base"));

    line_start = boost::any_cast<Eigen::VectorXd>(init.EndEffector[0].getProperty("BaseOffset")).head<3>();
    line_end = init.EndPoint.head<3>();

    line = line_end - line_start;
    infinite = init.infinite;

    if (debug_ && Server::isRos())
    {
        pub_marker = Server::advertise<visualization_msgs::MarkerArray>("p2l", 1000, true);
    }
}

int Point2Line::taskSpaceDim()
{
    return Kinematics[0].Phi.rows();
}
}  // namespace exotica
