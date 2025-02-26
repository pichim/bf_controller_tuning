def xspectrogram(x: list[float], y: list[float], Fs: float, NFFT: int = 200, resolution: int = 201, vmin: float = -50, vmax: float = 0, ax: plt.Axes = None, **kwargs):

    if ax is None:
        ax = plt.gca()

    noverlap: int = int(2/3 * NFFT)
    ymin: float = np.min(y)
    ymax: float = np.max(y)

    n = np.linspace(ymin, ymax, resolution)
    f = np.fft.rfftfreq(NFFT, 1/Fs)
    nf = len(f)

    spec: np.ndarray = np.zeros((resolution, nf), dtype=float)
    specCounter: np.ndarray = np.zeros(resolution, dtype=int)
    window: np.ndarray = np.hanning(NFFT)    

    ptr: int = 0
    ptrEnd: int = NFFT
    ptrDelta: int = int(NFFT - noverlap)

    while ptrEnd < len(x):

        yStart: int = int(ptr + (NFFT - ptrDelta)/2)
        yEnd: int = int(ptr + (NFFT + ptrDelta)/2)
        indices: list[int] = [round((y - ymin) / (ymax - ymin) * (resolution - 1)) for y in y[yStart:yEnd]]

        xWindowed: list[float] = np.multiply(window, x[ptr:ptrEnd])
        xMag: list[float] = np.abs(np.fft.rfft(xWindowed))

        for i in indices:
            spec[i] = np.add(spec[i], xMag)
            specCounter[i] += 1

        ptr += ptrDelta
        ptrEnd += ptrDelta

    for i in range(resolution):
        if specCounter[i] != 0:
            # psd = 20 * log10(2 * magnitude / bandwidth)
            spec[i] = 20 * np.log10(np.divide(spec[i], specCounter[i] * Fs/4))
        else:
            spec[i] = [vmin] * nf

    qmesh = ax.pcolormesh(n, f, np.transpose(spec), vmin=vmin, vmax=vmax, **kwargs)
    plt.colorbar(mappable=qmesh, ax=ax, pad=0.02, label='PSD (dB/Hz)')

    ax.set_ylabel('Frequency (Hz)')

    return qmesh