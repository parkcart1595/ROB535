import numpy as np

def ACC_Controller(t, x, param):
    vd = param["vd"]
    v0 = param["v0"]
    m = param["m"]
    Cag = param["Cag"]
    Cdg = param["Cdg"]

    # cost function and constraints for the QP
    P = np.zeros((2,2))
    q = np.zeros([2, 1])
    A = np.zeros([5, 2])
    b = np.zeros([5])
    
    #############################################################################
    #                    TODO: Implement your code here                         #
    #############################################################################

    # set the parameters
    # lam = ...
    # alpha = ...
    # w = ...

    # construct the cost function
    # P = ...
    # q = ...
    
    # construct the constraints
    # A = ...
    # b = ...

    #############################################################################
    #                            END OF YOUR CODE                               #
    #############################################################################
    
    return A, b, P, q