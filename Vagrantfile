Vagrant.configure("2") do |config|
  config.vm.box = "my-box"
  config.vm.network "private_network", ip: "192.168.50.4"
  config.vm.box_url = "https://repo.pg.innopolis.university/repository/vagrant-boxes/Debian-12.box"
  config.provision "shell", path: "provision.yml"
end
