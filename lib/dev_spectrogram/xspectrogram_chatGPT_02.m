% Function to compute a spectrogram-like display for two signals x and y
function qmesh = xspectrogram(x, y, Fs, NFFT, resolution, vmin, vmax, ax)
    % Check for input arguments and set defaults if not provided
    if nargin < 4 || isempty(NFFT)
        NFFT = 200;
    end
    if nargin < 5 || isempty(resolution)
        resolution = 201;
    end
    if nargin < 6 || isempty(vmin)
        vmin = -50;
    end
    if nargin < 7 || isempty(vmax)
        vmax = 0;
    end
    if nargin < 8 || isempty(ax)
        ax = gca; % Use current axes if none provided
    end

    % Calculate overlap size for windowing
    noverlap = floor(2/3 * NFFT);
    % Find the minimum and maximum values of y
    ymin = min(y);
    ymax = max(y);

    % Generate linearly spaced vector for the output y-axis
    n = linspace(ymin, ymax, resolution);
    % Compute the frequency bins for the output x-axis
    f = (0:NFFT/2) * (Fs / NFFT);
    % Number of frequency bins
    nf = length(f);

    % Initialize matrices for the spectrogram data and counter for averaging
    spec = zeros(resolution, nf);
    specCounter = zeros(resolution, 1);
    % Generate the windowing function
    window = hanning(NFFT);

    % Initialize pointers for traversing the signal
    ptr = 1;
    ptrEnd = NFFT;
    ptrDelta = NFFT - noverlap;

    % Process the signal in chunks
    while ptrEnd <= length(x)
        % Calculate the start and end indices for the y values
        yStart = round(ptr + (NFFT - ptrDelta)/2);
        yEnd = round(ptr + (NFFT + ptrDelta)/2);
        % Map y values to indices in the spectrogram resolution
        indices = round((y(yStart:yEnd) - ymin) / (ymax - ymin) * (resolution - 1)) + 1;

        % Window the x signal
        xWindowed = window .* x(ptr:ptrEnd);
        % Compute the magnitude of the FFT for the windowed signal
        xMag = abs(fft(xWindowed, NFFT));

        % Accumulate the magnitudes into the spectrogram matrix
        for i = 1:length(indices)
            idx = indices(i);
            spec(idx, :) = spec(idx, :) + xMag(1:nf).';
            specCounter(idx) = specCounter(idx) + 1;
        end

        % Advance the pointers for the next chunk
        ptr = ptr + ptrDelta;
        ptrEnd = ptrEnd + ptrDelta;
    end

    % Average the accumulated magnitudes and convert to dB scale
    for i = 1:resolution
        if specCounter(i) ~= 0
            spec(i, :) = 20 * log10(spec(i, :) ./ (specCounter(i) * Fs/4));
        else
            spec(i, :) = vmin * ones(1, nf); % Set to minimum value if no data
        end
    end

    % Plot the spectrogram data as a pseudocolor plot
    qmesh = pcolor(ax, f, n, spec);
    set(qmesh, 'EdgeColor', 'none'); % Remove gridlines
    clim(ax, [vmin vmax]); % Set the color axis scaling
    colorbar(ax, 'Position', [0.92 0.11 0.02 0.77]); % Add a colorbar
    xlabel(ax, 'Frequency (Hz)'); % Label the y-axis
end
