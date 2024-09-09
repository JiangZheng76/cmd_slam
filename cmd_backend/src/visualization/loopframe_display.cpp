#include "loopframe_display.hpp"
#include "loopframe.hpp"

namespace cmd
{
    static LoggerPtr g_logger_sys = SYLAR_LOG_NAME("system");

    LoopframeDisplay::LoopframeDisplay(LoopframePtr lf)
    {
        // TODO 替代 setKF 初始化整个 LoopframeDisplay
        m_num_sparse_points = 0;
        m_num_sparse_buffer_size = 0;

        m_active = true;

        m_need_refresh = false;
        m_buffer_valid = false;
        m_num_gl_buffer_points = 0;

        setFromLF(lf);
        m_lf = lf;
    }
    LoopframeDisplay::~LoopframeDisplay()
    {
    }
    void LoopframeDisplay::setFromLF(LoopframePtr lf)
    {
        MutextType::Lock lk(m_muetx);

        m_lf_id = lf->m_lf_id;
        // add all traces, inlier and outlier points.
        // 所有的已经边缘化的点
        int npoints = lf->m_points.size();

        if (m_original_point2s.size() < npoints)
        {
            m_original_point2s.resize(npoints);
        }
        auto &pc = m_original_point2s;
        m_num_sparse_points = 0;
        for (auto p : lf->m_points)
        {
            if (p.m_idepth_scaled <= 0)
                continue;
            pc[m_num_sparse_points] = p;
            m_num_sparse_points++;
        }
        m_Twc = lf->m_twc;
        SYLAR_ASSERT(m_num_sparse_points <= npoints);

        m_need_refresh = true;
    }

    void LoopframeDisplay::refreshPC()
    {
        if (!m_need_refresh)
        {
            return;
        }

        MutextType::Lock lk(m_muetx);
        m_need_refresh = false;
        // if there are no vertices, done!
        if (m_num_sparse_points == 0)
        {
            return;
        }

        // make data
        std::vector<Vec3Type> tempVertexBuf(m_num_sparse_points);
        std::vector<Vec3bType> tempColorBuf(m_num_sparse_points);

        precision_t fx = m_lf->m_calib.getFx();
        precision_t fy = m_lf->m_calib.getFy();
        precision_t cx = m_lf->m_calib.getCx();
        precision_t cy = m_lf->m_calib.getCy();

        for (int i = 0; i < m_num_sparse_points; i++)
        {
            precision_t depth =  1 / m_original_point2s[i].m_idepth_scaled;
            tempVertexBuf[i][0] =
                ((m_original_point2s[i].m_u - cx) / fx / m_original_point2s[i].m_idepth_scaled);
            tempVertexBuf[i][1] =
                ((m_original_point2s[i].m_v - cy) / fy / m_original_point2s[i].m_idepth_scaled);
            tempVertexBuf[i][2] =
                depth * (1 + (2 / fx) * (rand() / (float)RAND_MAX - 0.5f));

            tempColorBuf[i][0] = static_cast<unsigned char>(m_color[0] * 255);
            tempColorBuf[i][1] = static_cast<unsigned char>(m_color[1] * 255);
            tempColorBuf[i][2] = static_cast<unsigned char>(m_color[2] * 255);
        }

        m_num_gl_buffer_good_points = m_num_sparse_points;
        if (m_num_gl_buffer_good_points > m_num_gl_buffer_points)
        {
            // 扩充 pangolin buf 的空间
            m_num_gl_buffer_points = m_num_sparse_points;
            m_vertex_buffer.Reinitialise(pangolin::GlArrayBuffer, m_num_gl_buffer_points,
                                         GL_DOUBLE, 3, GL_DYNAMIC_DRAW);
            m_color_buffer.Reinitialise(pangolin::GlArrayBuffer, m_num_gl_buffer_points,
                                        GL_DOUBLE, 3, GL_DYNAMIC_DRAW);
        }
        // 点和 color 内容写到 glbuf 中
        m_vertex_buffer.Upload(&tempVertexBuf[0],
                               sizeof(precision_t) * 3 * m_num_gl_buffer_good_points, 0);
        m_color_buffer.Upload(&tempColorBuf[0],
                              sizeof(unsigned char) * 3 * m_num_gl_buffer_good_points, 0);

        m_buffer_valid = true;
    }

    void LoopframeDisplay::drawPC(float pointSize)
    {

        if (!m_buffer_valid || m_num_gl_buffer_good_points == 0)
            return;

        glDisable(GL_LIGHTING);
        glColor3f(m_color[0], m_color[1], m_color[2]);
        glPushMatrix();

        // 确定画点的位置
        Sophus::Matrix4f m = m_Twc.matrix().cast<float>();
        glMultMatrixf((GLfloat *)m.data());
        // 画出来点的大小
        glPointSize(pointSize);
        // size：指定每个颜色的分量数量。通常为 3（表示 RGB 颜色）或 4（表示 RGBA 颜色）。
        // type：指定颜色数据的类型。通常使用 GL_UNSIGNED_BYTE（每个分量为无符号字节），GL_FLOAT（每个分量为浮点数）等。
        // stride：指定相邻颜色之间的字节偏移量。通常可以设置为 0，表示颜色数据是紧密排列的。
        // pointer：指向颜色数据的指针。这可以是一个数组、缓冲区对象或其他内存位置。

        // m_vertex_buffer.Bind();
        // glVertexPointer(m_vertex_buffer.count_per_element, m_vertex_buffer.datatype, 0, 0);
        // glEnableClientState(GL_VERTEX_ARRAY);
        // glDrawArrays(GL_POINTS, 0, m_num_gl_buffer_good_points);
        // glDisableClientState(GL_VERTEX_ARRAY);
        // m_vertex_buffer.Unbind();
        //
        // glPopMatrix();
        m_vertex_buffer.Bind();
        glVertexPointer(m_vertex_buffer.count_per_element, m_vertex_buffer.datatype, 0, 0);
        glEnableClientState(GL_VERTEX_ARRAY);

        // m_color_buffer.Bind();
        // glColorPointer(m_color_buffer.count_per_element, m_color_buffer.datatype, 0, 0);
        // glEnableClientState(GL_COLOR_ARRAY);

        glDrawArrays(GL_POINTS, 0, m_num_gl_buffer_good_points);

        glDisableClientState(GL_VERTEX_ARRAY);
        glDisableClientState(GL_COLOR_ARRAY);
        m_vertex_buffer.Unbind();
        // m_color_buffer.Unbind();

        glPopMatrix();
    }

    void LoopframeDisplay::initColor(std::vector<float> color)
    {
        SYLAR_ASSERT(color.size() == 3);
        m_color = color;
    }

}