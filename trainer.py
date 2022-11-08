# TODO first implement ArenaTrainer, then a generalized trainer
# TODO implement DiscoveryTrainer

# TODO implement it so you can use multiple trainers simultaneously
# TODO each trainer when trainer.step() is called, pass a flag whether it can start an active training (e.g. rolling) process,
#   also every trainer returns whether it is currently running active training

# TODO can create a MultiTrainer class that can incorporate any trainers, and their priorities, so if 2 trainers
#   want to train simultaneously, then one is preferred over the other; also it can weight/sum rewards coming from all...
