package name.drahflow.ar.apps.VNC;

class ConnectionBeanStorage {
	private String address;
	private String password;
	private String username;
	private String repeaterId;
	private String nickname;
	private String colorModel;
	private String inputMode;
	private String scaleMode;
	private boolean useRepeater;
	private boolean useLocalCursor;
	private boolean keepPassword;
	private int port;
	private int id;
	private long forceFull;

	public void setAddress(String address) { this.address = address; }
	public String getAddress() { return this.address; }

	public void setPort(int port) { this.port = port; }
	public int getPort() { return this.port; }

	public void set_Id(int id) { this.id = id; }
	public int get_Id() { return this.id; }

	public void setUserName(String username) { this.username = username; }
	public String getUserName() { return this.username; }

	public void setPassword(String password) { this.password = password; }
	public String getPassword() { return this.password; }

	public void setRepeaterId(String repeaterId) { this.repeaterId = repeaterId; }
	public String getRepeaterId() { return this.repeaterId; }

	public void setNickname(String nickname) { this.nickname = nickname; }
	public String getNickname() { return this.nickname; }

	public void setColorModel(String colorModel) { this.colorModel = colorModel; }
	public String getColorModel() { return this.colorModel; }

	public void setInputMode(String inputMode) { this.inputMode = inputMode; }
	public String getInputMode() { return this.inputMode; }

	public void setScaleModeAsString(String scaleMode) { this.scaleMode = scaleMode; }
	public String getScaleModeAsString() { return this.scaleMode; }

	public void setUseRepeater(boolean useRepeater) { this.useRepeater = useRepeater; }
	public boolean getUseRepeater() { return this.useRepeater; }

	public void setUseLocalCursor(boolean useLocalCursor) { this.useLocalCursor = useLocalCursor; }
	public boolean getUseLocalCursor() { return this.useLocalCursor; }

	public void setKeepPassword(boolean keepPassword) { this.keepPassword = keepPassword; }
	public boolean getKeepPassword() { return this.keepPassword; }

	public void setForceFull(long forceFull) { this.forceFull = forceFull; }
	public long getForceFull() { return this.forceFull; }
}
