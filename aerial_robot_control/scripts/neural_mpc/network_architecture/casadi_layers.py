import torch
import casadi as ca

class caLinear(torch.nn.Linear):
    def ca_forward(self, x):
        assert x.shape[1] == 1, "Casadi can not handle batches."
        y = ca.mtimes(self.weight.detach().cpu().numpy(), x)
        if self.bias is not None:
            y = y + self.bias.detach().cpu().numpy()
        return y
    
class caBatchNorm1D(torch.nn.BatchNorm1d):
    def ca_forward(self, x):
        """
        Normalize the input as in PyTorch but with casadi operations.
        """
        assert x.shape[1] == 1, "Casadi can not handle batches."

        # Get the parameters as numpy arrays
        running_mean = self.running_mean.detach().cpu().numpy()
        running_var = self.running_var.detach().cpu().numpy()
        weight = self.weight.detach().cpu().numpy()
        bias = self.bias.detach().cpu().numpy()

        # BatchNorm1d normalization formula from PyTorch documentation:
        # output = (input - running_mean) / sqrt(running_var + eps) * weight + bias
        normalized = (x - running_mean) / ca.sqrt(running_var + self.eps)
        output = normalized * weight + bias

        return output
    
class caDropout(torch.nn.Dropout):
    def ca_forward(self, x):
        """
        Apply dropout to the input tensor using CasADi operations.
        """
        return x  # Dropout is deactivated while using eval()
    

# Activation functions
class caSigmoid(torch.nn.Sigmoid):
    def ca_forward(self, x):
        y = 1 / (1 + ca.exp(-x))
        return y


class caTanh(torch.nn.Tanh):
    def ca_forward(self, x):
        return ca.tanh(x)


class caReLU(torch.nn.ReLU):
    def ca_forward(self, x):
        return ca.if_else(x < 0.0, 0.0 * x, x)


class caLeakyReLU(torch.nn.LeakyReLU):
    def ca_forward(self, x):
        return ca.if_else(x < 0.0, self.negative_slope * x, x)


class caGELU(torch.nn.GELU):
    def ca_forward(self, x):
        """
        Exact GELU, matching torch.nn.GELU()'s default (approximate='none').

        This used to return the tanh approximation
            0.5*x*(1 + tanh(sqrt(2/pi)*(x + 0.044715*x^3)))
        which is what torch computes only with approximate='tanh'. The two
        differ by up to 4.7e-4 per activation, so the model embedded in the MPC
        was systematically NOT the model that was trained — measured on
        neuralmodel_209 the residual output differed by 1.8e-4 m/s².

        That is negligible against a ~1 m/s² residual, but it is a train/deploy
        mismatch, and with online learning the two models are supposed to be the
        same object: the trainer minimises the error of the torch model while
        the controller flies the CasADi one. ca.erf matches torch to 4e-16, has
        an analytic derivative, and code-generates to C99 erf() — so there is no
        reason to keep the approximation.
        """
        return 0.5 * x * (1 + ca.erf(x / ca.sqrt(2)))
