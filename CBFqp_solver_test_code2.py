import numpy as np
from cvxopt import matrix, solvers

class QPSolverCAV:
    def __init__(self):
        self.u_min = -10  # Minimum control input (deceleration)
        self.u_max = 1  # Maximum control input (acceleration)
        self.phiRearEnd = 0.8  # Reaction time for rear-end safety constraint
        self.phiLateral = 1  # Reaction time for lateral safety constraint
        self.deltaSafetyDistance = 0.25  # Minimum safety distance (meters)
        self.v_min = 0  # Minimum velocity
        self.v_max = 1  # Maximum velocity
        self.x0 = [0.0, 0.0, 0.0, 0.0]  # Initial state: [d0, vel, d2, d3]
        self.state = [0.0, 0.0, 0.0, -1, -1, -1, -1, 0.0, -1, -1]  # State variables: [ID, vel, d0, d1, v1, d2, v2, vd, d3, v3]

    def set_state(self, state):
        self.state = state
        self.x0 = [state[2], state[1], state[5], state[8]]

    def OCBF_SecondOrderDynamics(self):
        x0 = self.x0  # x0[0] is d0, x0[1] is vel, x0[2] is d2, x0[3] is d3
        eps = 10
        psc = 0.1
        vd = self.state[7]

        # Reference control input
        u_ref = 1

        # Physical limitations on velocity
        b_vmax = self.v_max
        b_vmin = self.v_min

        # CLF
        phi0 = -eps * (x0[1] - vd) ** 2
        phi1 = 2 * (x0[1] - vd)

        # Initial A and b matrices
        A = np.array([[1, 0], [-1, 0], [phi1, -1], [1, 0]])
        b = np.array([self.u_max, -self.u_min, phi0, b_vmax])

        # Rear-end Safety Constraints
        if self.state[3] != -1:
            print("Applying rear-end CBF")
            d1 = self.state[3]
            h = d1 - self.phiRearEnd * x0[1] - self.deltaSafetyDistance
            vip = self.state[4]  # Velocity of the preceding vehicle
            LgB = self.phiRearEnd
            LfB = vip - x0[1]
            if LgB != 0:
                A = np.append(A, [[LgB, 0]], axis=0)
                b = np.append(b, [LfB + h])
            print("cbfreatr", (LfB+h)/LgB)
        # Lateral Safety Constraints
        lateral_constraints = [(self.state[5], self.state[6]), (self.state[8], self.state[9])]  # (d2, v2), (d3, v3)
        L = 3.5  # Length of the merging lane

        for (d, v) in lateral_constraints:
            if d != -1:
                print("Applying lateral CBF")
                bigPhi = self.phiLateral * x0[0] / L
                h = d - bigPhi * x0[1] - self.deltaSafetyDistance
                LgB = bigPhi
                LfB = v - x0[1] - self.phiLateral * x0[1] ** 2 / L
                if LgB != 0:
                    A = np.append(A, [[LgB, 0]], axis=0)
                    b = np.append(b, [LfB + h])
                    print("cbf", (LfB+h)/LgB)

        # QP formulation
        H = matrix([[1, 0], [0, psc]], tc='d')
        f = matrix([[-u_ref], [0]], (2, 1), tc='d')  # Ensure f is a 2x1 column vector
        A = matrix(A, tc='d')
        b = matrix(b, tc='d')

        solvers.options['show_progress'] = False
        try:
            Solution = solvers.qp(H, f, A, b)
            u = Solution['x'][0]
        except ValueError as e:
            print(f"QP solver error: {e}")
            u = self.u_min

        return u

    def recalc_qp(self):
        if self.state is not None:
            u = self.OCBF_SecondOrderDynamics()
            print("Computed control input u:", u)
            return u

# Test cases
if __name__ == '__main__':
    solver = QPSolverCAV()

    # Test case 1: Only rear-end CBF
    state1 = ["A", 0.5, 0.1, 0.1, 0.5, -1, -1, 1, -1, -1]  # [ID, vel, d0, d1, v1, d2, v2, vd, d3, v3]
    solver.set_state(state1)
    print("Test case 1 - Only rear-end CBF")
    solver.recalc_qp()
    print("\n")

    # Test case 2: Only first lateral CBF
    state2 = ["A", 0.3, 0.1, -1, -1, 0.5, 0.4, 0.5, -1, -1]  # [ID, vel, d0, d1, v1, d2, v2, vd, d3, v3]
    solver.set_state(state2)
    print("Test case 2 - Only first lateral CBF")
    solver.recalc_qp()
    print("\n")

    # Test case 3: Both rear-end and first lateral CBF
    state3 = ["A", 0.1, 1.0, 0.5, 1, 0.5, 0.4, 0.5, -1, -1]  # [ID, vel, d0, d1, v1, d2, v2, vd, d3, v3]
    solver.set_state(state3)
    print("Test case 3 - Both rear-end and first lateral CBF")
    solver.recalc_qp()
    print("\n")

    # Test case 4: Rear-end, first lateral, and second lateral CBF
    state4 = ["A", 0.5, 1.0, 0.2, 0.4, 0.5, 0.4, 0.5, 1, 0.4]  # [ID, vel, d0, d1, v1, d2, v2, vd, d3, v3]
    solver.set_state(state4)
    print("Test case 4 - Rear-end, first lateral, and second lateral CBF")
    solver.recalc_qp()
    print("\n")

    # Test case 5: Only two lateral CBFs
    state5 = ["A", 0.3, 0.1, -1, -1, 0.05, 0.4, 0.5, 1, 0.4]  # [ID, vel, d0, d1, v1, d2, v2, vd, d3, v3]
    solver.set_state(state5)
    print("Test case 5 - Only two lateral CBFs")
    solver.recalc_qp()
    print("\n")
