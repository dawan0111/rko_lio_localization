# MIT License
#
# Copyright (c) 2025 Meher V.R. Malladi.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

FROM ros:jazzy-perception
# do i get env variables from the shell in the build? i dont think so
ENV ROS_DISTRO=jazzy
# ubuntu 24.04 images include an ubuntu user by default which has GID 1000
RUN userdel -r ubuntu

RUN apt-get update && apt-get upgrade -y \
  && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install --no-install-recommends -y \
  zsh \
  git \
  python3-pip \
  python3-dev \
  python3-setuptools \
  python-is-python3 \
  tmux \
  sudo \
  curl \
  wget \
  build-essential \
  tree \
  openssh-client \
  software-properties-common \
  ninja-build \
  gettext \
  unzip \
  libgmp-dev \
  libmpfr-dev \
  libmpc-dev \
  libisl-dev \
  zlib1g-dev \
  file \
  cmake \
  libssl-dev \
  ros-${ROS_DISTRO}-rosbag2-storage-mcap \
  ros-${ROS_DISTRO}-rosbag2 \
  ros-${ROS_DISTRO}-foxglove-bridge \
  ros-${ROS_DISTRO}-rmw-zenoh-cpp \
  ros-${ROS_DISTRO}-rviz2 \
  ros-${ROS_DISTRO}-plotjuggler-ros \
  libeigen3-dev \
  libtbb-dev \
  && rm -rf /var/lib/apt/lists/*

WORKDIR /tmp

# neovim
ENV NEOVIM_VERSION=0.11.2
RUN wget https://github.com/neovim/neovim/archive/refs/tags/v${NEOVIM_VERSION}.tar.gz && \
  tar -zxf v${NEOVIM_VERSION}.tar.gz && \
  rm v${NEOVIM_VERSION}.tar.gz && \
  cd /tmp/neovim-${NEOVIM_VERSION} && \
  make CMAKE_BUILD_TYPE=Release && \
  make install && \
  rm -rf /tmp/neovim-${NEOVIM_VERSION} 

ARG UID=1000
ARG GID=1000
ENV UNAME=dev

# Add normal sudo-user to container, passwordless, taken from nacho's ros in docker
RUN addgroup --gid $GID $UNAME \
  && adduser --disabled-password --gecos '' --uid $UID --gid $GID $UNAME \
  && adduser $UNAME sudo \
  && echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers \
  && sed -i 's/required/sufficient/' /etc/pam.d/chsh \
  && touch /home/$UNAME/.sudo_as_admin_successful

WORKDIR /home/${UNAME}/
ENV HOME=/home/${UNAME}
USER ${UNAME}
ENV PATH="${PATH}:${HOME}/.local/bin"

# once it gets merged in, use the master
# RUN sh -c "$(wget -O- https://github.com/deluan/zsh-in-docker/releases/download/v1.2.1/zsh-in-docker.sh)" -- \
RUN sudo sh -c "$(curl -L https://raw.githubusercontent.com/balancedscorpion/zsh-in-docker/new_user/zsh-in-docker.sh)" -- \
  -u ${UNAME} \
  -x \
  -p git -p z -p sudo -p history -p copybuffer -p copypath -p copyfile \
  -p https://github.com/zsh-users/zsh-autosuggestions \
  -p https://github.com/zsh-users/zsh-completions

RUN sudo chsh -s $(which zsh) ${UNAME}
ENV SHELL=/usr/bin/zsh
SHELL ["/bin/zsh", "-lc"]

RUN echo "export _TYPER_STANDARD_TRACEBACK=1" >> /home/${UNAME}/.zshrc \
  && echo "export EDITOR=nvim" >> /home/${UNAME}/.zshrc \
  && echo "alias vim=nvim" >> /home/${UNAME}/.zshrc \
  && echo "export HISTFILE=/home/${UNAME}/.zsh_history" >> /home/${UNAME}/.zshrc \
  && echo "ros-env() {" >> /home/${UNAME}/.zshrc \
  && echo "    source /opt/ros/${ROS_DISTRO}/setup.zsh" >> /home/${UNAME}/.zshrc \
  && echo "    source /home/${UNAME}/ros_ws/install/setup.zsh 2>/dev/null || true" >> /home/${UNAME}/.zshrc \
  && echo "}" >> /home/${UNAME}/.zshrc \
  && echo "ros-env" >> /home/${UNAME}/.zshrc \
  && echo 'export ROS_LANG_DISABLE=genlisp:gennodejs:geneus' >> /home/${UNAME}/.zshrc 

# rustup
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y \
  && echo 'export PATH="$HOME/.cargo/bin:$PATH"' >> /home/${UNAME}/.zshrc
RUN cargo install --locked tree-sitter-cli

# nvm
RUN curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.40.2/install.sh | bash \
  && export NVM_DIR="$([ -z "${XDG_CONFIG_HOME-}" ] && printf %s "${HOME}/.nvm" || printf %s "${XDG_CONFIG_HOME}/nvm")" \
  && . $NVM_DIR/nvm.sh \
  && nvm install node \
  && nvm use node \
  && nvm alias default node \
  && echo 'export NVM_DIR="$HOME/.nvm"' >> /home/${UNAME}/.zshrc \
  && echo '[ -s "$NVM_DIR/nvm.sh" ] && \\. "$NVM_DIR/nvm.sh"  # This loads nvm' >> /home/${UNAME}/.zshrc

# dotfiles
RUN git clone --depth=1 https://github.com/mehermvr/dotfiles.git /tmp/dotfiles \
  && mkdir -p /home/${UNAME}/.config/tmux \
  && cp -r /tmp/dotfiles/.config/nvim /home/${UNAME}/.config/nvim \
  && cp -r /tmp/dotfiles/.config/tmux/tmux.conf /home/${UNAME}/.tmux.conf \
  && rm -rf /tmp/dotfiles

RUN git clone --depth 1 https://github.com/junegunn/fzf.git /home/${UNAME}/.fzf && /home/${UNAME}/.fzf/install --all

# sets up tmux plugins
RUN git clone https://github.com/tmux-plugins/tpm /home/${UNAME}/.tmux/plugins/tpm \
  && tmux start-server \
  && tmux new-session -d \
  && /home/${UNAME}/.tmux/plugins/tpm/tpm \
  && /home/${UNAME}/.tmux/plugins/tpm/scripts/install_plugins.sh \
  && tmux kill-server

# sets up all the lazy plugins
RUN nvim --headless "+Lazy! sync" +qa

WORKDIR /home/${UNAME}/ros_ws
CMD ["/bin/zsh", "-i"]
