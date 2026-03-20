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
        return 0.5 * x * (1 + ca.tanh(ca.sqrt(2 / ca.pi) * (x + 0.044715 * ca.power(x, 3))))
