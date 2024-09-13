#!/usr/bin/env python3
import rospy
import numpy as np
from cav_project.msg import limo_info, limo_info_array, QP_solution
from cvxopt import matrix, solvers

QP_SOLN = QP_solution()

class QPSolver:
    def __init__(self):
        rospy.init_node("QP_solver_node")
        self.qp_solution_pub = rospy.Publisher('/qp_solution', QP_solution, queue_size=10)
        self.cav_info_sub = rospy.Subscriber('/cav_info', limo_info, self.cav_info_callback)
        self.other_cav_info_sub = rospy.Subscriber('/other_cav_info', limo_info_array, self.other_cav_info_callback)
        self.cav_info = None
        self.other_cav_info = None
        self.rate = rospy.Rate(10)
        self.u_min = -10
        self.deltaSafetyDistance = 300

    def cav_info_callback(self, msg):
        self.cav_info = msg

    def other_cav_info_callback(self, msg):
        self.other_cav_info = msg

    def solve_qp(self, qp_mat, my_vec):
        ocpar = [-0.593787660013256, 1.41421356237309, 0, 0, 2.38168230431317, 1.68410370801184]
        c = np.array(ocpar)
        x0 = np.array(my_vec)
        eps = 10
        psc = 0.1
        t = 0.1

        vd = 1
        u_ref = c[0] * t + c[1]
        u_ref = 1

        b_vmax = 1 - x0[1]
        b_vmin = x0[1] - 0.15

        phi0 = -eps * (x0[1] - vd) ** 2
        phi1 = 2 * (x0[1] - vd)

        A = np.array([[1, 0], [-1, 0], [phi1, -1], [1, 0]])
        b = np.array([3, -10, phi0, b_vmax])

        if qp_mat[0][0] != -1:
            xip = qp_mat[0][1]
            h = xip - x0[2] - 1.2 * x0[1] - self.deltaSafetyDistance
            vip = qp_mat[0][2]
            LgB = 1.2
            LfB = vip - x0[1]
            if LgB != 0:
                A = np.append(A, [[LgB, 0]], axis=0)
                b = np.append(b, [LfB + h])

        for row_index, row in enumerate(qp_mat):
            if row_index == 0:
                continue
            if -1 in row:
                continue
            else:
                d1 = qp_mat[row_index][3]
                d2 = my_vec[row_index + 2]
            L = my_vec[row_index + 2] + my_vec[0]

            v0 = qp_mat[row_index][2]
            bigPhi = 1.0 * x0[0] / L
            h = d2 - d1 - bigPhi * x0[1] - self.deltaSafetyDistance

            uminValue = abs(self.u_min)
            hf = d2 - d1 - 0.5 * (v0 - x0[1]) ** 2 / uminValue - 1.0 * v0 * (x0[0] + 0.5 * (x0[1] ** 2 - v0 ** 2) / uminValue) / L

            LgB = bigPhi
            LfB = v0 - x0[1] - 1.0 * x0[1] ** 2 / L
            if LgB != 0:
                A = np.append(A, [[LgB, 0]], axis=0)
                b = np.append(b, [LfB + h])

        H = matrix([[1, 0], [0, psc]])
        f = matrix([[-u_ref], [0]], (2,1), 'd')
        H = matrix(H, tc='d')
        f = matrix(f, tc='d')
        A = matrix(A, tc='d')
        b = matrix(b, tc='d')

        solvers.options['show_progress'] = False
        Solution = solvers.qp(H, f, A, b)
        if Solution['status'] == 'optimal':
            return Solution['x'].trans()
        return None

    def recalc_QP(self):
        infeasible = False
        if not self.cav_info or not self.other_cav_info:
            return

        QP_rows = [[-1, -1, -1, -1]]
        d2s = [-1]

        for other_cav in self.other_cav_info.limo_infos:
            if other_cav.ID.data > 0:
                if other_cav.d2.data >= 0:
                    d2s.append(other_cav.d2.data)
                    row = [
                        other_cav.ID.data,
                        other_cav.origin_dist.data,
                        other_cav.vel.data,
                        other_cav.d1.data
                    ]
                    if other_cav.d1.data + self.deltaSafetyDistance > other_cav.d2.data:
                        infeasible = True
                else:
                    d2s.append(-1)
                    row = [-1, -1, -1, -1]
                QP_rows.append(row)
            else:
                QP_rows[0] = [
                    other_cav.ID.data,
                    other_cav.d1.data,
                    other_cav.vel.data,
                    other_cav.origin_dist.data
                ]
                d2s[0] = other_cav.d2.data

        my_vec = [self.cav_info.origin_dist.data, self.cav_info.vel.data] + d2s
        my_vec = np.array(my_vec)
        qp_mat = np.array(QP_rows)
        u = None

        if len(qp_mat) > 1 and not infeasible:
            u = self.solve_qp(qp_mat, my_vec)
        if u is None:
            u = self.u_min
        QP_SOLN.u.data = u
        self.qp_solution_pub.publish(QP_SOLN)

    def run(self):
        while not rospy.is_shutdown():
            if self.cav_info and self.other_cav_info:
                self.recalc_QP()
            self.rate.sleep()

if __name__ == '__main__':
    solver = QPSolver()
    solver.run()
