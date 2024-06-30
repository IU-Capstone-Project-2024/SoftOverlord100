#!/usr/bin/env bash

sudo apt-get update
# Add Docker's official GPG key:
sudo apt-get install ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/debian/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc

# Add the repository to Apt sources:
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/debian \
  $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

sudo apt-get update
sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

usermod -aG docker vagrant

### Install GitLab Runner
# Download the binary for your system
sudo curl -L --output /usr/local/bin/gitlab-runner https://gitlab-runner-downloads.s3.amazonaws.com/latest/binaries/gitlab-runner-linux-amd64

# Give it permission to execute
sudo chmod +x /usr/local/bin/gitlab-runner

# Create a GitLab Runner user
sudo useradd --comment 'GitLab Runner' --create-home gitlab-runner --shell /bin/bash
sudo usermod -aG docker gitlab-runner

# Install and run as a service
sudo gitlab-runner install --user=gitlab-runner --working-directory=/home/gitlab-runner
sudo gitlab-runner start

RUNNER_TOKEN="glrt-UXaXfNCPWA5Vjz_qm2pG"

sudo gitlab-runner register \
  --non-interactive \
  --url "https://gitlab.pg.innopolis.university" \
  --token "$RUNNER_TOKEN" \
  --executor "shell" \
  --description "Development environment"

# Read https://docs.gitlab.com/runner/shells/index.html#shell-profile-loading
# to know why comment bash_logout
sed -i '/^\([^#]\)/s/^/# /g' /home/gitlab-runner/.bash_logout
