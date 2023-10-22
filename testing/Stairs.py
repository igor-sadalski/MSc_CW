#policy evaluation
class Stairs:

    def __init__(self, gamma):
        self.gamma = gamma

    def policyEvaluation(self):
        '''compute equiprobable policy'''

        def policy(action: int, state: int) -> float:
            prob = 1/2
            assert 0 < prob < 1, "probability must be in range [0,1]"
            return prob

        def reward(action, state):
            next_state = state + action
            if next_state == 6:
                return 10
            elif next_state == 0:
                return -10
            else:
                return -action
            
        def iterativeInPlaceBellman(V: np.ndarray, curr_state: int,
                            gamma: float) -> float:
            new_val = 0
            for action in [-1, 1]: #only two actions
                pi = policy(action, curr_state)
                r = reward(action, curr_state)
                vn = V[curr_state + action]
                new_val +=  pi * (r + gamma * vn)
            return new_val

        theta = 0.001 #change treshold to stop iteration
        n_states = 7
        V = np.zeros((n_states)) #initial empty array of values at states
        max_it = 50 #iteration treshold  
        it = 0 
        while it < max_it:
            dlt = 0 #change between two consequtive states
            for curr_state in range(1, n_states-1):
                v = V[curr_state]
                V[curr_state] = iterativeInPlaceBellman(V, curr_state, self.gamma) #try to impelement with two vectors
                dlt = max(dlt, abs(v - V[curr_state])) #always save maximal change of values
            it += 1
            print('{} iteration'.format(it))
            print(np.round(V,2))
            if dlt < theta:
                break
            self.V = V
        #optional
        # return V



    def policyImprovement(self) -> np.ndarray:
        '''0 - go left, 1 - go right'''
        V = self.V
        out = np.zeros(7)
        for i in range(1, len(V)-1):
            if V[i-1] > V[i+1]:
                out[i] = 0
            elif V[i-1] < V[i+1]:
                out[i] = 1
            else:
                out[i] = random_choice = np.random.choice([0, 1])
        print(out)
        # return out

    def analyticalSolution(self):
        p = np.zeros((7,7))
        p[0][0] = 1
        p[6][6] = 1
        ind = 0
        for i in range(1,6):
            p[i][ind] = 1/2
            p[i][ind+2] = 1/2
            ind += 1
        print("transition probability | equiprobable policy")
        print(p)

        r = np.zeros((7,7))
        ind = 0
        for i in range(1,6):
            r[i][ind] = 1
            r[i][ind+2] = -1
            ind += 1
        r[1][0] = -10
        r[6][6] = 10
        print('reward | equiprobable policy')
        print(r)
        val = np.round(np.linalg.inv(np.eye(7) - gamma * p) @ r, 1)
        print('value state matrix??? \n', val)
        # return val

#TO DO: policy improvement
import numpy as np
gamma = 0.9
a = Stairs(gamma)
a.policyEvaluation()
a.policyImprovement()
a.analyticalSolution()