function [plotFigure, metrics] = exportBodeTeachingFigure(loopModel, loopLabel, outputBase)
%EXPORTBODETEACHINGFIGURE Export an unstyled MATLAB native margin plot.
%   Keep MATLAB's automatic margin title, default curves and characteristics.
    arguments
        loopModel (1, 1) {mustBeA(loopModel, ["tf", "ss", "zpk"])}
        loopLabel (1, 1) string
        outputBase (1, 1) string
    end
    assert(issiso(loopModel), "BodeTeaching:NotSISO", "Provide a SISO loop model.");
    [gainMargin, phaseMargin, gainCrossOmega, crossOmega] = margin(loopModel);
    assert(isfinite(crossOmega) && crossOmega > 0, ...
        "BodeTeaching:NoCrossover", "A finite positive crossover is required.");
    crossHz = crossOmega/(2*pi);
    plotFigure = figure(Name=loopLabel, NumberTitle="off");
    % Compute margins on the full model; zoom only the displayed axes.
    margin(loopModel);
    bodeChart = findobj(plotFigure, "Type", "bode");
    % Retain native rad/s units so R2025a margin markers stay correctly placed.
    upperOmega = 3*crossOmega;
    if isfinite(gainCrossOmega)
        upperOmega = max(upperOmega, 1.2*gainCrossOmega);
    end
    displayOmega = logspace(log10(crossOmega/10), log10(upperOmega), 1800);
    [displayMagnitude, displayPhase] = bode(loopModel, displayOmega);
    magnitudeLimits = [min(-10, floor(min(20*log10(displayMagnitude(:)))/10)*10), ...
        max(10, ceil(max(20*log10(displayMagnitude(:)))/10)*10)];
    phaseLimits = [min(-180, floor(min(displayPhase(:))/45)*45), ...
        ceil(max(displayPhase(:))/45)*45];
    setoptions(bodeChart, 'XLim', [crossOmega/10, upperOmega], ...
        'YLim', {magnitudeLimits; phaseLimits});
    grid(bodeChart, "on");
    drawnow;
    outputFolder = fileparts(outputBase);
    if ~isfolder(outputFolder)
        mkdir(outputFolder);
    end
    exportgraphics(plotFigure, outputBase + ".png", Resolution=200);
    print(plotFigure, outputBase + ".svg", "-dsvg", "-vector");
    savefig(plotFigure, outputBase + ".fig");

    frequencyHz = logspace(log10(crossHz/10), log10(3*crossHz), 1800);
    response = reshape(freqresp(loopModel, 2*pi*frequencyHz), 1, []);
    magnitudeDb = 20*log10(abs(response));
    phaseDeg = unwrap(angle(response))*180/pi;
    samples = table(frequencyHz(:), magnitudeDb(:), phaseDeg(:), ...
        VariableNames=["frequency_hz", "magnitude_db", "phase_deg"]);
    writetable(samples, outputBase + "_response.csv");
    metrics = table(loopLabel, crossHz, phaseMargin, 20*log10(gainMargin), phaseMargin-180, ...
        VariableNames=["loop", "crossover_hz", "phase_margin_deg", "gain_margin_db", "phase_at_crossover_deg"]);
end
