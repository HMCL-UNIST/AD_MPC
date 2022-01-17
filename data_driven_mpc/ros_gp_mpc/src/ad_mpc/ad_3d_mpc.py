""" Implementation of the data-augmented MPC for quadrotors.

This program is free software: you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation, either version 3 of the License, or (at your option) any later
version.
This program is distributed in the hope that it will be useful, but WITHOUT
ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
You should have received a copy of the GNU General Public License along with
this program. If not, see <http://www.gnu.org/licenses/>.
"""


import numpy as np
from ad_mpc.ad_3d_optimizer import AD3DOptimizer
from model_fitting.gp_common import restore_gp_regressors
from utils.quad_3d_opt_utils import simulate_plant, uncertainty_forward_propagation
from utils.utils import make_bx_matrix


class AD3DMPC:
    def __init__(self, my_ad, t_horizon=1.0, n_nodes=20, q_cost=None, r_cost=None,
                 optimization_dt=5e-2, simulation_dt=5e-4, model_name="my_ad", solver_options=None):    
        """
        :param my_ad: AD 2D simulator object
        :type my_ad: AD 2D model
        :param t_horizon: time horizon for optimization loop MPC controller
        :param n_nodes: number of MPC nodes
        :param optimization_dt: time step between two successive optimizations intended to be used.
        :param simulation_dt: discretized time-step for the quadrotor simulation        
        :param q_cost: diagonal of Q matrix for LQR cost of MPC cost function. Must be a numpy array of length 13.
        :param r_cost: diagonal of R matrix for LQR cost of MPC cost function. Must be a numpy array of length 4.        
        :param solver_options: Optional set of extra options dictionary for acados solver.        
        """
        self.ad = my_ad
        self.simulation_dt = simulation_dt
        self.optimization_dt = optimization_dt
        # input commands from last step
        self.pre_u = np.array([0., 0.])

        self.n_nodes = n_nodes
        self.t_horizon = t_horizon

        # For MPC optimization use
        self.ad_opt = AD3DOptimizer(my_ad, t_horizon=t_horizon, n_nodes=n_nodes,
                                        q_cost=q_cost, r_cost=r_cost,                                        
                                        model_name=model_name, 
                                        solver_options=solver_options)

    def clear(self):
        self.ad_opt.clear_acados_model()

    def get_state(self):
        """
        Returns the state of the AD
        :return: 4x1 array with the state: [p_xy,psi,vel]
        """
        x = np.expand_dims(self.ad.get_state(stacked=True), 1)
        return x

    def set_reference(self, x_reference, u_reference=None, terminal_point = False):
        """
        Sets a target state for the MPC optimizer
        :param x_reference: list with 4 sub-components (position, angle quaternion, velocity, body rate). If these four
        are lists, then this means a single target point is used. If they are Nx3 and Nx4 (for quaternion) numpy arrays,
        then they are interpreted as a sequence of N tracking points.
        :param u_reference: Optional target for the optimized control inputs
        """

        if isinstance(x_reference[0], list) or terminal_point:
            # Target state is just a point
            return self.ad_opt.set_reference_state(x_reference, u_reference)
        else:
            # Target state is a sequence
            return self.ad_opt.set_reference_trajectory(x_reference, u_reference)

    def optimize(self, use_model=0, return_x=False):
        """
        Runs MPC optimization to reach the pre-set target.
        :param use_model: Integer. Select which dynamics model to use from the available options.
        :param return_x: bool, whether to also return the optimized sequence of states alongside with the controls.

        :return: 2*m vector of optimized control inputs with the format: [u_1(0), u_2(0), u_1(1), ...,
        u_1(m-1), u_2(m-1)]. If return_x is True, will also return a vector of shape N+1 x 4 containing the optimized
        state prediction.
        """
        # ad_current_state = self.ad.get_state(stacked=True)     
        ad_current_state = np.expand_dims(self.ad.get_state(stacked=True), 0)   

        # Remove rate state for simplified model NLP
        out_out = self.ad_opt.run_optimization(ad_current_state, use_model=use_model, return_x=return_x)
        return out_out

       
    @staticmethod
    def reshape_input_sequence(u_seq):
        """
        Reshapes the an output trajectory from the 1D format: [u_0(0), u_1(0), ..., u_0(n-1), u_1(n-1), ..., u_m-1(n-1)]
        to a 2D n x m array.
        :param u_seq: 1D input sequence
        :return: 2D input sequence, were n is the number of control inputs and m is the dimension of a single input.
        """
        k = np.arange(u_seq.shape[0] / 4, dtype=int)
        u_seq = np.atleast_2d(u_seq).T if len(u_seq.shape) == 1 else u_seq
        u_seq = np.concatenate((u_seq[4 * k], u_seq[4 * k + 1], u_seq[4 * k + 2], u_seq[4 * k + 3]), 1)
        return u_seq

    def reset(self):
        return
