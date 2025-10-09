# IGVC Software 2026

## Development

### Autocomplete

Some shells such as ZSH have issues by default with ros2 autocomplete. Add the following commands to your `~/.zshrc` to enable auto complete.
```bash
eval "$(register-python-argcomplete ros2)"
eval "$(register-python-argcomplete colcon)"
```
