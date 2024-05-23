#include "loopframe_display.hpp"
#include "loopframe.hpp"

namespace cmd
{
    static LoggerPtr g_logger_sys = SYLAR_LOG_NAME("system");

    LoopframeDisplay::LoopframeDisplay(LoopframePtr lf)
    {
        // TODO 替代 setKF 初始化整个 LoopframeDisplay
        m_num_sparse_buffer_size = 0;
        m_num_sparse_points = 0;

        m_client_lf_id = 0;
        m_active = true;

        m_need_refresh = false;

        m_my_scaled_th = 0.001;
        m_my_abs_th = 0.001;
        m_my_displayMode = 1;
        m_my_min_rel_bs = 0.1;
        m_my_sparsify_factor = 1;

        m_num_gl_buffer_points = 0;
        m_buffer_valid = false;

        setFromLF(lf);
        m_lf = lf;
    }
    LoopframeDisplay::~LoopframeDisplay()
    {
        
    }
    void LoopframeDisplay::setFromLF(LoopframePtr lf)
    {
        MutextType::Lock lk(m_muetx);
        // add all traces, inlier and outlier points.
        // 所有的已经边缘化的点
        int npoints = lf->m_points.size();

        if (m_original_input_sparse.size() < npoints)
        {
            m_original_input_sparse.resize(npoints + 100);
        }
        std::vector<InputPointSparse<MAX_RES_PER_POINT>> &pc = m_original_input_sparse;
        m_num_sparse_points = 0;
        for (auto p : lf->m_points)
        {
            if (p.m_idepth_scaled <= 0)
                continue;
            for (int i = 0; i < patternNum; i++)
            {
                // 暂时不知道这个color有什么作用？？？
                pc[m_num_sparse_points].color[i] = i;
            }
            pc[m_num_sparse_points].u = p.m_u;
            pc[m_num_sparse_points].v = p.m_v;
            pc[m_num_sparse_points].idpeth = p.m_idepth_scaled;
            // 表示已经被边缘化的点
            pc[m_num_sparse_points].status = 1;
            // 下面的作用未知？？？
            pc[m_num_sparse_points].idepth_hessian = p.m_idepth_hessian;
            pc[m_num_sparse_points].relObsBaseline = p.m_maxRelBaseline;
            pc[m_num_sparse_points].numGoodRes = 0;
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
        std::vector<Vec3Type> tempVertexBuf(m_num_sparse_points * patternNum);
        std::vector<Vec3bType> tempColorBuf(m_num_sparse_points * patternNum);
        int vertexBufferNumPoints = 0;

        for (int i = 0; i < m_num_sparse_points; i++)
        {
            /* display modes:
             * my_displayMode_==0 - all pts, color-coded
             * my_displayMode_==1 - normal points
             * my_displayMode_==2 - active only
             * my_displayMode_==3 - nothing
             */
            // 显示状态为1 同时点状态为 1和 2 的点可以显示出来
            if (m_my_displayMode == 1 && m_original_input_sparse[i].status != 1 &&
                m_original_input_sparse[i].status != 2)
                continue;
            if (m_my_displayMode == 2 && m_original_input_sparse[i].status != 1)
                continue;
            if (m_my_displayMode > 2)
                continue;

            if (m_original_input_sparse[i].idpeth < 0)
                continue;

            float depth = 1.0f / m_original_input_sparse[i].idpeth;
            float depth4 = depth * depth;
            depth4 *= depth4;
            float var = (1.0f / (m_original_input_sparse[i].idepth_hessian + 0.01));

            if (var * depth4 > m_my_scaled_th)
                continue;

            if (var > m_my_abs_th)
                continue;

            if (m_original_input_sparse[i].relObsBaseline < m_my_min_rel_bs)
                continue;

            for (int pnt = 0; pnt < patternNum; pnt++)
            {

                if (m_my_sparsify_factor > 1 && rand() % m_my_sparsify_factor != 0)
                    continue;
                int dx = patternP[pnt][0];
                int dy = patternP[pnt][1];

                tempVertexBuf[vertexBufferNumPoints][0] =
                    ((m_original_input_sparse[i].u + dx) * m_fxi + m_cxi) * depth;
                tempVertexBuf[vertexBufferNumPoints][1] =
                    ((m_original_input_sparse[i].v + dy) * m_fxi + m_cxi) * depth;
                tempVertexBuf[vertexBufferNumPoints][2] =
                    depth * (1 + 2 * m_fxi * (rand() / (float)RAND_MAX - 0.5f));

                tempColorBuf[vertexBufferNumPoints][0] = m_color[0];
                tempColorBuf[vertexBufferNumPoints][1] = m_color[1];
                tempColorBuf[vertexBufferNumPoints][2] = m_color[2];

                vertexBufferNumPoints++;

                SYLAR_ASSERT(vertexBufferNumPoints <= m_num_sparse_points * patternNum);
            }
        }

        if (vertexBufferNumPoints == 0)
        {
            // std::cout << "refresh loopframe pointNum " << vertexBufferNumPoints << std::endl;
            SYLAR_LOG_INFO(g_logger_sys) << m_client_lf_id << "没有刷新点.";
            return;
        }

        m_num_gl_buffer_good_points = vertexBufferNumPoints;
        if (m_num_gl_buffer_good_points > m_num_gl_buffer_points)
        {
            // 扩充 pangolin buf 的空间
            m_num_gl_buffer_points = vertexBufferNumPoints * 1.3;
            m_vertex_buffer.Reinitialise(pangolin::GlArrayBuffer, m_num_gl_buffer_points,
                                         GL_FLOAT, 3, GL_DYNAMIC_DRAW);
            m_color_buffer.Reinitialise(pangolin::GlArrayBuffer, m_num_gl_buffer_points,
                                        GL_UNSIGNED_BYTE, 3, GL_DYNAMIC_DRAW);
        }
        // 点和 color 内容写到 glbuf 中
        m_vertex_buffer.Upload(&tempVertexBuf[0],
                               sizeof(float) * 3 * m_num_gl_buffer_good_points, 0);
        m_color_buffer.Upload(&tempColorBuf[0],
                              sizeof(unsigned char) * 3 * m_num_gl_buffer_good_points,
                              0);

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
        Sophus::Matrix4d m = m_Twc.matrix();
        glMultMatrixd((GLdouble *)m.data());
        // 画出来点的大小
        glPointSize(pointSize);
        // 这个绑定颜色不行，使用glColor来对点进行染色
        // glColorPointer()函数
        // size：指定每个颜色的分量数量。通常为 3（表示 RGB 颜色）或 4（表示 RGBA 颜色）。
        // type：指定颜色数据的类型。通常使用 GL_UNSIGNED_BYTE（每个分量为无符号字节），GL_FLOAT（每个分量为浮点数）等。
        // stride：指定相邻颜色之间的字节偏移量。通常可以设置为 0，表示颜色数据是紧密排列的。
        // pointer：指向颜色数据的指针。这可以是一个数组、缓冲区对象或其他内存位置。

        m_vertex_buffer.Bind();
        glVertexPointer(m_vertex_buffer.count_per_element, m_vertex_buffer.datatype, 0, 0);
        glEnableClientState(GL_VERTEX_ARRAY);
        glDrawArrays(GL_POINTS, 0, m_num_gl_buffer_good_points);
        glDisableClientState(GL_VERTEX_ARRAY);
        m_vertex_buffer.Unbind();

        glPopMatrix();
    }
    void LoopframeDisplay::initColor(std::vector<float> color)
    {
        SYLAR_ASSERT(color.size() == 3);
        m_color = color;
    }

}