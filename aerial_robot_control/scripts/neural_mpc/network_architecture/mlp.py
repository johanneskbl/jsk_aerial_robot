import torch
from network_architecture.casadi_layers import caLinear, caBatchNorm1D, caDropout, caReLU, caLeakyReLU, caSigmoid, caTanh, caGELU


class MLP(torch.nn.Module):
    def __init__(
        self,
        in_size,
        hidden_sizes,
        out_size,
        activation="relu",
        use_batch_norm=False,
        dropout_p=0.0,
        dropout_input=False,
        x_mean=None,
        x_std=None,
        y_mean=None,
        y_std=None,
    ):
        super().__init__()
        assert len(hidden_sizes) >= 1, "There must be at least one hidden layer"

        layers = []
        prev_size = in_size
        if dropout_input:
            layers.append(caDropout(dropout_p))
        for i in range(len(hidden_sizes) + 1):  # +1 to compensate for input layer
            if i < len(hidden_sizes):
                next_size = hidden_sizes[i]
            else:
                next_size = hidden_sizes[-1]  # Repeat last hidden layer size
            # Fully connected layer
            layers.append(caLinear(prev_size, next_size))
            # Batch normalization
            if use_batch_norm:
                layers.append(caBatchNorm1D(next_size))
            # Activation function
            if activation == "ReLU":
                layers.append(caReLU())
            elif activation == "Tanh":
                layers.append(caTanh())
            elif activation == "Sigmoid":
                layers.append(caSigmoid())
            elif activation == "LeakyReLU":
                layers.append(caLeakyReLU())
            elif activation == "GELU":
                layers.append(caGELU())
            elif activation is None:
                pass  # Equal to lambda x: x
            else:
                raise ValueError(f"Unsupported activation function: {activation}")
            # Dropout
            if dropout_p > 0.0:
                layers.append(caDropout(dropout_p))

            prev_size = next_size

        layers.append(caLinear(prev_size, out_size))

        self.fully_connected_stack = torch.nn.ModuleList(layers)

        # Input-Output Normalization
        self.register_buffer("x_mean", x_mean)
        self.register_buffer("x_std", x_std)
        self.register_buffer("y_mean", y_mean)
        self.register_buffer("y_std", y_std)

        # For ML Casadi library
        self.input_size = in_size
        self.output_size = out_size

    def forward(self, x):
        # Input normalization
        x = (x - self.x_mean) / self.x_std
        # Forward pass
        for layer in self.fully_connected_stack:
            x = layer(x)
        # Output denormalization
        return (x * self.y_std) + self.y_mean

    def ca_forward(self, x):
        # Input normalization
        x = (x - self.x_mean.cpu().numpy()) / self.x_std.cpu().numpy()
        # Forward pass
        for layer in self.fully_connected_stack:
            x = layer.ca_forward(x)
        # Output denormalization
        return (x * self.y_std.cpu().numpy()) + self.y_mean.cpu().numpy()
