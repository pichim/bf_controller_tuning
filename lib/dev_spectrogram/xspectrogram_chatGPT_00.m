function qmesh = xspectrogram(x, y, Fs, NFFT, resolution, vmin, vmax, ax)

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
        ax = gca;
    end

    noverlap = floor(2/3 * NFFT);
    ymin = min(y);
    ymax = max(y);

    n = linspace(ymin, ymax, resolution);
    f = (0:NFFT/2) * (Fs / NFFT);
    nf = length(f);

    spec = zeros(resolution, nf);
    specCounter = zeros(resolution, 1);
    window = hanning(NFFT);

    ptr = 1;
    ptrEnd = NFFT;
    ptrDelta = NFFT - noverlap;

    while ptrEnd < length(x)

        yStart = floor(ptr + (NFFT - ptrDelta)/2);
        yEnd = floor(ptr + (NFFT + ptrDelta)/2);
        indices = round((y(yStart:yEnd) - ymin) / (ymax - ymin) * (resolution - 1)) + 1;

        xWindowed = window .* x(ptr:ptrEnd);
        xMag = abs(fft(xWindowed, NFFT));
        xMag = xMag(1:nf);

        for idx = 1:length(indices)
            if indices(idx) > 0 && indices(idx) <= resolution
                spec(indices(idx), :) = spec(indices(idx), :) + xMag';
                specCounter(indices(idx)) = specCounter(indices(idx)) + 1;
            end
        end

        ptr = ptr + ptrDelta;
        ptrEnd = ptrEnd + ptrDelta;
    end

    for i = 1:resolution
        if specCounter(i) ~= 0
            spec(i, :) = 20 * log10(spec(i, :) / (specCounter(i) * Fs/4));
        else
            spec(i, :) = vmin * ones(1, nf);
        end
    end

    [N, F] = meshgrid(n, f);
    qmesh = pcolor(ax, N, F, spec');
    set(qmesh, 'EdgeColor', 'none');
    caxis(ax, [vmin vmax]);
    colorbar(ax, 'Position', [0.92 0.11 0.02 0.77]);

    ylabel(ax, 'Frequency (Hz)');

end