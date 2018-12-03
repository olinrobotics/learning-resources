# Pseudocode
class KalmanFilter():
  def __init__(self, prior):
    
    # Initialize prior with gaussian passed in
    pass
  
  def predict(self, posterior, movement):
    
    x, P = posterior # Unpack posterior gaussian
    dx, Q = movement # Unpack movement gaussian
    
    x = x + dx       # Update mean w/ predicted movement
    P = P + Q        # Combine variances, decrease certainty
    
    # Return gaussian(x,P)
    pass
  
  def update(self, prior, measurement):
    
    x, P = prior       # Unpack prior gaussian
    z, R = measurement # Unpack measurement gaussian
    K = P/(P+R)        # Calculate Kalman Gain from variances
    x = x + K(z-x)     # Update mean w/ weighted avg of measurement and prior
    P = (1-K)*P        # Update variance, increase certainty
    
    # Return gaussian(x,P)
    pass
  
  def step(self, z, dx, R=R_0, Q=Q_0):
    measurement = gaussian(z, R)
    movement = gaussian(dx, Q)
    # prior = predict(prior, movement)
    # posterior = update(prior, measurement)
    # self.prior = posterior
    # Return posterior
    pass

# Pseudocode
class Robot():
  
  def __init__(self, data_file):
    
    # Initialize Kalman filter
    # Initialize subscribers
      # /enc
      # /cmd_vel
      # /ground_truth
      # /imu
      # /laser
    
    # Initialize prev_encoder_val (wait until encoder data)
    # Initialize encoder position (start at 0,0)
    # Initialize current command (start with 0,0)
    # Initialize kalman filter
    
    # Setup writing to csv file data_file
      # Check if file exists (prevent overwriting)
      # Open file
      # Source: https://stackabuse.com/reading-and-writing-csv-files-in-python/
    
    pass
  
  def encoder_cb(self, msg):
    
    # update encoder position based on prev_encoder_val
    pass
  
  def commandvel_cb(self, msg):
    
    # update current command
    pass
  
  def run(self):
    
    # Run kalman filter loop; save new estimate
    # Record estimates:
      # kalman filter
      # encoder model
      # ground truth 
    # Maintain constant frequency (rate.sleep)

    pass

