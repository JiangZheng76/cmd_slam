#include "Comm/vo_comm.hpp"

#include "FullSystem/HessianBlocks.h"

#include "util/FrameShell.h"

DSVComm::DSVComm(std::string ip, std::string port)
    : cmd::FrontEndComm(ip, port)
{
}

void DSVComm::publishLoopframe(dso::FrameHessian *fh, dso::CalibHessian *HCalib,
                                          precision_t dso_error, precision_t scale_error)
{
    if (cur_id_ >= fh->frameID) {
      return;
    }
    cur_id_ = fh->frameID;
    static int s_id = 0;
    MsgLoopframePtr msg(new MsgLoopframe());
    msg->m_lf_id = s_id++;
    msg->m_client_id = m_client_id;
    msg->m_incoming_id = fh->shell->id;
    msg->m_twc = fh->shell->camToWorld.matrix();
    msg->m_is_update_msg = false;
    msg->m_timestamp = fh->shell->timestamp;

    msg->m_img_x_min = 0;
    msg->m_img_y_min = 0;
    msg->m_img_x_max = dso::wG[0];
    msg->m_img_y_max = dso::hG[0];

    msg->m_ab_exposure = fh->ab_exposure;
    msg->m_dso_error = dso_error;
    msg->m_scale_error = scale_error;

    msg->m_calib.setIntrinsics(HCalib->fxl(), HCalib->fyl(), HCalib->cxl(), HCalib->cyl());
    msg->m_calib.m_img_dims = {dso::wG[0], dso::hG[0]};
    msg->m_calib.m_dist_coeffs = {0, 0, 0, 0};

    msg->m_msg_points.reserve(fh->pointHessiansMarginalized.size());
    for (auto ph : fh->pointHessiansMarginalized)
    {
        cmd::MsgPoint point;
        point.m_u = ph->u;
        point.m_v = ph->v;
        point.m_idepth_scaled = ph->idepth_scaled;
        point.m_maxRelBaseline = ph->maxRelBaseline;
        point.m_idepth_hessian = ph->idepth_hessian;
        msg->m_msg_points.push_back(point);
    }

    publishMsg(msg);
}