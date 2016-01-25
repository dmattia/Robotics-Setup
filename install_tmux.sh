echo "Checking if tmux installed..."
if ! which tmux > /dev/null;
then
	echo "Installing tmux..."
	sudo apt-get install tmux
	echo "Success: tmux installed"
else
	echo "tmux is already installed"
fi
